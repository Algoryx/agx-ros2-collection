# AGX ROS2 collection
Collection of ROS2 packages that with libraries, tutorials and resources for using ROS2 together with AGX Dynamics.
Clone this repository into your workspace/src directory and build running the ``colcon build`` command from your workspace directory.
Created by Algoryx Simulation AB under the Apache-2.0 license.

## agx_tutorials
This packages collects tutorials for how to use ROS2 packages like ros2_control and moveIt together with AGX Dynamics, AGX Dynamics for Unity and AGX Dynamics for Unreal.

### Franka Emika Panda robot
Launch the tutorial with:

`ros2 launch agx_tutorials panda_demo_launch.py`

It will start the AGX Dynamics simulation of the a Panda robot together with MoveIt and ros2_control with the `JointTrajectoryController` for follow the trajectories planned using MoveIt. In this example the command interface
for the robot joint is position, meaning that the controller just forwards the positions to the robot joints. The
robot joints holds the assigned position using the [Lock1D](https://www.algoryx.se/documentation/complete/agx/tags/latest/doc/UserManual/source/constraints.html#lock1d) secondary constraint.

![Panda](images/panda.jpg)

The same example exists using effort as command interface. Start the effort example using:

`ros2 launch agx_tutorials panda_demo_effort_launch.py`

The controller will now try to follow the planned trajectory position by setting efforts (torques) on the joints of the simulated robot. The error in the position is mapped to efforts through a PID-loop. The simulated robot is also gravity compensated using AGX Dynamics inverse dynamics module.

There is also an launch file to launch the ROS2 stack without starting the simulation.

`ros2 launch agx_tutorials panda_demo_no_simulation_launch.py`

Then you must have launched AGX simulation somewhere else first. This can for example be useful when you have the simulation running in AGX Dynamics for Unity or AGX Dynamics for Unreal (examples for this is coming), or if you are on windows running the simulation natively in windows and the ros2 stack using WSL2.

#### Running on Windows
To run the simulation on windows checkout this repository and source your windows AGX installation.
When in the root of the repository (where this README.md is) run the .bat file `start_panda_simulation.bat position` to start the simulation with the position command interface or `start_panda_simulation.bat effort` to start the simulation with the effort command interface. After you have started the simulation you can open your WSL2 terminal and source the workspace where you have built these tutorial packages and run 

`ros2 launch agx_tutorials panda_demo_no_simulation_launch.py`

The ROS2 stack will with MoveIt and ros2_control will run in WSL and speak ROS2 with the simulation running on windows.


### More robots to come
...


### Note on connection between ros2_control and AGX Simulation
ros2_control controllers are speaking to the AGX Simulation using the ROS2 package [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control). The robot listens to `sensor_msgs/JointState` commands on the `agx_joint_commands` topic and sends the current joint states back on the `agx_joint_states` topic. This is currently asynchronous and sensitive to that the simulation runs in realtime. 

## Sending and receiving custom data types

By using the ``AnyMessageBuilder`` and ``AnyMessageParser`` it is possible to serialize and deserialize custom data types at runtime and send them as an ``agx_msgs::Any`` message.
To see an example of this, first build the ``agx_msgs``, ``agx_any_builder_parser`` and ``agx_any_msg_example`` packages.
Source the workspace you built these packages in and then run the sender example:

    > ros2 run agx_any_msg_example sender

In another terminal, source ROS2, your workspace and then run the receiver example:

    > ros2 run agx_any_msg_example receiver
