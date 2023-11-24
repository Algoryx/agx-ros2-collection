#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_in_square");

using namespace geometry_msgs::msg;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true); // set 
  auto const node = std::make_shared<rclcpp::Node>(
    "move_in_square",
    options
  );
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  // SingleThreadedExecutor for the current state monitor to get information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_in_square");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  move_group_interface.setPlanningTime(20.0);
  std::vector<Pose> waypoints;
  Pose start_pose = move_group_interface.getCurrentPose().pose;
  waypoints.push_back(start_pose);

  RCLCPP_INFO(LOGGER, "start pose %.2f, %.2f, %.2f", start_pose.position.x, start_pose.position.y, start_pose.position.z);

  Pose target_pose = start_pose;
  target_pose.position.y -= 0.15;
  waypoints.push_back(target_pose);

  target_pose.position.x += 0.3;
  waypoints.push_back(target_pose);

  target_pose.position.y += 0.3;
  waypoints.push_back(target_pose);

  target_pose.position.x -= 0.3;
  waypoints.push_back(target_pose);

  target_pose.position.y -= 0.15;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  if (fraction >= 1.0)
    move_group_interface.execute(trajectory);

  // Go back home  
  move_group_interface.setNamedTarget("ready");
  move_group_interface.move();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}