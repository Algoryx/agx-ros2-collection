import agx
import agxROS2
import agxSDK
import agxOSG
import agxCollide
import agxRender
import agxModel

import sys
import random
import argparse

from agxPythonModules.utils.environment import simulation, root, application, init_app
from agxPythonModules.utils.callbacks import StepEventCallback as sec

from agxROS2 import ROS2ControlInterface as cpp_ROS2ControlInterface, ROS2ClockPublisher

import numpy as np


def setupCamera(app):
    camera_data = app.getCameraData()
    camera_data.eye = agx.Vec3(1.1645, 0.7695, 1.2471)
    camera_data.center = agx.Vec3(0.1959, -0.1783, 0.3564)
    camera_data.up = agx.Vec3(-0.4020, -0.3745, 0.8356)
    camera_data.nearClippingPlane = 0.1
    camera_data.farClippingPlane = 5000
    app.applyCameraData(camera_data)


def objects_and_table(material):
    # table 
    table_size = agx.Vec3(0.2)
    table = agxCollide.Geometry(agxCollide.Box(table_size))
    table.setPosition(0.4, 0.0, table_size.z())
    simulation().add(table)
    node = agxOSG.createVisual(table, root())
    agxOSG.setTexture(node, "grid.png")

    delta = 0.04

    def create_box(pos, name, material):
        size = agx.Vec3(0.02)
        box = agx.RigidBody(name)
        geom = agxCollide.Geometry(agxCollide.Box(size))
        geom.setMaterial(material)
        box.setPosition(pos + agx.Vec3(0, 0, size.z()))
        box.add(geom)
        return box
    
    colors = [
        (agxRender.Color_Blue(), "Blue"),
        (agxRender.Color_Red(), "Red"),
        (agxRender.Color_Green(), "Green"),
        (agxRender.Color_BlanchedAlmond(), "BlanchedAlmond")]
    
    table_pos = table.getPosition()
    pos = [
        (table_pos.x()-table_size.x() + 2*delta, -table_size.y() + 2*delta),
        (table_pos.x()+table_size.x() - 2*delta, -table_size.y() + 2*delta),
        (table_pos.x()-table_size.x() + 2*delta, table_size.y() - 2*delta),
        (table_pos.x()+table_size.x() - 2*delta, table_size.y() - 2*delta),
    ]
    objects = []
    for p in pos:
        color = random.choice(colors)
        o = create_box(agx.Vec3(p[0], p[1], 2*table_size.z()), f"{color[1]}_box", material)
        # o.getMassProperties().setMass(0.01)
        simulation().add(o)
        print(o)
        node = agxOSG.createVisual(o, root())
        agxOSG.setDiffuseColor(node, color[0])
        objects.append(o)
    return objects

def setup_gravity_comp(sim, control_joint_names):
    passive_joints = agx.ConstraintContainer()
    control_joints = agx.ConstraintContainer()
    
    for c in sim.getConstraints():
        if c.getName() in control_joint_names:
            control_joints.append(agx.ConstraintRef(c))
        else:
            passive_joints.append(agx.ConstraintRef(c))
    
    ids = agxModel.InverseDynamics(sim, sim.getRigidBodies(), control_joints, passive_joints)
    def gravity_compensation(t):
        ids.sync(agxModel.InverseDynamics.FULL)
        ids.gravityCompensation()
        forces = ids.getJointForces(0)
        for f in forces:
            joint = control_joints[f.jointIndex]
            rep = joint.getRep()
            motor = rep.getMotor1D(f.angleIndex)
            lock = rep.getLock1D(f.angleIndex)
            # Control via force/torque:
            # Disable the motor (if it happens to be enabled)
            motor.setEnable(False)
            # Enable the lock
            lock.setEnable(True)
            # By setting the lower and upper value of the ForceRange to the same value,
            # the result is that a constant force/torque is applied.
            lock.setForceRange(f.value, f.value)
    sec.preCallback(gravity_compensation)

def buildScene1():
    ap = argparse.ArgumentParser()
    ap.add_argument("panda-urdf", help="Path to the panda.urdf file to load")
    ap.add_argument("panda-urdf-package", help="Path to the package directory that the panda.urdf file references")
    ap.add_argument("--command-interface", default="position", choices=["position", "effort"], help="Effort control requires this fix for ros2_controllers https://github.com/ros-controls/ros2_controllers/pull/558")
    args, _ = ap.parse_known_args()
    args = vars(args)

    # Construct the floor that the panda robot will stand on
    floor = agxCollide.Geometry(agxCollide.Box(agx.Vec3(5, 5, 0.1)))
    floor.setPosition(0, 0, -0.11)
    simulation().add(floor)
    fl = agxOSG.createVisual(floor, root())
    agxOSG.setDiffuseColor(fl, agxRender.Color.LightGray())

    # Set the sky color
    application().getSceneDecorator().setBackgroundColor(agxRender.Color.SkyBlue(), agxRender.Color.DodgerBlue())

    # Read the URDF file
    urdf_file = args["panda-urdf"]
    package_path = args["panda-urdf-package"]

    # Determines if the base link of the URDF model should be attached to the
    # world or not. Default is false.
    fixToWorld = True

    # If true the collision between linked/constrained bodies will be disabled.
    # Default is false.
    disableLinkedBodies = True

    # If true, any link missing the <inertial></inertial> key will be treated
    # as a kinematic link and will be merged with its parent. According to http://wiki.ros.org/urdf/XML/link
    # If false, the body will get its mass properties calculated from the shape/volume/density.
    # Default is True.
    mergeKinematicLink = True

    settings = agxModel.UrdfReaderSettings(fixToWorld, disableLinkedBodies, mergeKinematicLink)

    init_joint_angles = agx.RealVector()
    init_joint_angles.append(0.0)
    init_joint_angles.append(-0.785)
    init_joint_angles.append(0.0)
    init_joint_angles.append(-2.356)
    init_joint_angles.append(0.0)
    init_joint_angles.append(1.571)
    init_joint_angles.append(0.785)
    init_joint_angles.append(0.0)
    init_joint_angles.append(0.0)
    panda_assembly_ref = agxModel.UrdfReader.read(urdf_file, package_path, init_joint_angles, settings)
    if (panda_assembly_ref.get() is None):
        print("Error reading the URDF file.")
        sys.exit(2)
    panda = panda_assembly_ref.get()

    # Disable self-collision between links.
    simulation().getSpace().setEnablePair(panda_assembly_ref.getName(), panda_assembly_ref.getName(), False)

    # Add the panda assembly to the simulation and create visualization for it
    simulation().add(panda)
    fl = agxOSG.createVisual(panda, root())

    ros2_clock = ROS2ClockPublisher()
    simulation().add(ros2_clock, agxSDK.EventManager.HIGHEST_PRIORITY)

    command_interface = cpp_ROS2ControlInterface.POSITION
    if args["command_interface"] == "position":
        command_interface = cpp_ROS2ControlInterface.POSITION
    elif args["command_interface"] == "effort":
        command_interface = cpp_ROS2ControlInterface.EFFORT

    control_joint_names = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ]
    if command_interface == cpp_ROS2ControlInterface.EFFORT:
        setup_gravity_comp(simulation(), control_joint_names)

    panda_arm_control_interface = cpp_ROS2ControlInterface(
        "agx_joint_commands",
        "agx_joint_states",
        syncWithSystemTime=True
    )
    for name in control_joint_names:
        if not panda_arm_control_interface.addJoint(simulation().getConstraint1DOF(name), command_interface):
            print("Could not add joint ", name)
    # only need to add the first finger since the other is setup to mimic the motion in the physics model
    if not panda_arm_control_interface.addJoint(simulation().getConstraint1DOF("panda_finger_joint1"), cpp_ROS2ControlInterface.POSITION):
        print("Could not add finger")

    simulation().add(panda_arm_control_interface)

    gripper_material = agx.Material("gripper")
    objects = objects_and_table(gripper_material)

    for g in simulation().getRigidBody("panda_rightfinger").getGeometries():
        g.setMaterial(gripper_material)
    for g in simulation().getRigidBody("panda_leftfinger").getGeometries():
        g.setMaterial(gripper_material)
    
    finger_pose_pub = agxROS2.PublisherGeometryMsgsPose2D("fingerPose")
    def publish_finger_pose(t):
        msg = agxROS2.GeometryMsgsPose2D()
        msg.x = simulation().getRigidBody("panda_rightfinger").getPosition().x()
        msg.y = simulation().getRigidBody("panda_rightfinger").getPosition().y()
        finger_pose_pub.sendMessage(msg)
    sec.preCallback(publish_finger_pose)

    # cm
    cm = simulation().getMaterialManager().getOrCreateContactMaterial(gripper_material, gripper_material)
    cm.setFrictionCoefficient(1.0)
    fm = agx.IterativeProjectedConeFriction()
    fm.setSolveType(agx.FrictionModel.DIRECT)
    cm.setFrictionModel(fm)

    simulation().setTimeStep(0.004)
    # Setup the camera
    setupCamera(application())
    application().getSceneDecorator().setEnableShadows(True)
    return root()


def buildScene():
    buildScene1()


# Entry point when this script is started with python executable
init = init_app(name=__name__,
                scenes=[(buildScene, '1')],
                autoStepping=True,    # Default. False
                onInitialized=lambda application: print('App successfully initialized.'),
                onShutdown=lambda application: print('App successfully shut down.'))
