#include <memory>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initializes ROS 2 client library
  rclcpp::init(argc, argv);
  // Create a shared_ptr to the Node, which is required by MoveGroupInterface
  // and allows it to access parameters and the ROS graph.
  // The NodeOptions are set to automatically declare parameters from overrides.
  // This is useful for testing with different parameters without changing the code
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveGroupInterface instance
  // This interface allows you to interact with the MoveIt! planning interface.
  // The first argument is the Node, and the second is the name of the planning group.
  // In this case, "ur_manipulator" is the name of the robot's planning group
  // since is is tied to a ROS2 node it can use ROS communication (actions, topics, etc.
  // internally subsvcribes to the /joint_states and tf topic)
  // and loads the robot description from the parameter server.
  // also connects to ur_manipulator/move_action for planning
  // and ur_manipulator/execute_action for executing plans
  // Note: MoveGroupInterface is a class that provides an interface to the MoveIt! planning
  // interface, allowing you to set goals, plan paths, and execute motions.
  // sets up state monitoring and kinematics for the ur_manipulator planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_node = MoveGroupInterface(node, "ur_manipulator");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();

  // Set the target pose for the MoveGroupInterface
  //converts the target pose to a format that MoveGroupInterface can use
  // this entails converting the geometry_msgs::msg::Pose to a format into a goal constraints message 
  // this include: the pose, the target frame (usually the end effector frame), and any tolerance settings
  // then stores it in the MoveGroupInterface object
  // goal_constraints field becomes part of the next move_action call
  move_group_node.setPoseTarget(target_pose);




  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_node]{

    // i think this is a MoveGroupInterface::Plan object which contains the trajectory to be executed
    // is included the vurrent state of the robot, the planned trajectory, and any other relevant information
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    // sends a request to the MoveGroupInterface to plan a trajectory to the target pose
    // this internally calls the move_action server to plan a trajectory
    // the MoveGroupInterface::Plan object is filled with the planned trajectory
    // the MoveGroupInterface::Plan object is returned as a pair with a boolean indicating success
    // this is done using the move_group_node.plan(msg) method
    // the move_group_node loads the planning pipeline, performs IK, checks for collisions, gernerates a trajectory,
    // and returns the plan in the msg object
    // if the planning is successful, the msg object will contain a valid trajectory
    auto const ok = static_cast<bool>(move_group_node.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    // The plan was successful, so we can execute it
    // internally calls the ur_manipulator/execute_action server to execute the trajectory
    // it forwards to trajectory to trajectory_execution_manager which publushes it to the /joint_trajectory_action topic
    // the joint__trajectory_controller interports the trajectory and sends it to the robot
    move_group_node.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}