#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2_eigen/tf2_eigen.hpp>  // Fixed: Use .hpp instead of .h for Humble

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <iostream> // Added for terminal input
#include <chrono>   // Added for sleep

/** Compute tool0 pose for a desired TCP pose using standard MoveIt 2 pattern */
geometry_msgs::msg::Pose tool0PoseFromTcpPose(const geometry_msgs::msg::Pose& tcp_in_world)
{
  // Constant rigid transform from tool0 to your TCP (= "camera_tip")
  // 5 cm offset in x, y, z directions (units in meters)
  // Fixed: Proper Eigen Isometry3d construction
  static const Eigen::Isometry3d T_tool0_tcp = 
        Eigen::Isometry3d(Eigen::Translation3d(0.05, 0.05, 0.05)) *
        Eigen::Isometry3d(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d T_world_tcp;
  tf2::fromMsg(tcp_in_world, T_world_tcp);

  // Standard robotics transform: T_world_tool0 = T_world_tcp * (T_tool0_tcp)^-1
  Eigen::Isometry3d T_world_tool0 = T_world_tcp * T_tool0_tcp.inverse();
  return tf2::toMsg(T_world_tool0);
}

/** Get the current TCP pose in world frame */
geometry_msgs::msg::Pose getCurrentTcpPose(const moveit::planning_interface::MoveGroupInterface& move_group)
{
  // TCP offset transform (same as above) - Fixed: Proper Eigen conversion
  static const Eigen::Isometry3d T_tool0_tcp = 
        Eigen::Isometry3d(Eigen::Translation3d(0.05, 0.05, 0.05));

  // Get current tool0 pose
  auto current_state = move_group.getCurrentState();
  const Eigen::Isometry3d& T_world_tool0 = current_state->getGlobalLinkTransform("tool0");
  
  // Calculate TCP pose: T_world_tcp = T_world_tool0 * T_tool0_tcp
  Eigen::Isometry3d T_world_tcp = T_world_tool0 * T_tool0_tcp;
  return tf2::toMsg(T_world_tcp);
}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  RCLCPP_INFO(logger, "=== Starting MoveIt TCP Demo ===");
  RCLCPP_INFO(logger, "Step 1: Creating ROS executor thread...");

  // CRITICAL: We spin up a SingleThreadedExecutor so MoveItVisualTools can interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(logger, "Step 2: Creating MoveGroup interface...");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  RCLCPP_INFO(logger, "Step 3: Creating MoveIt Visual Tools...");

  // Create the MoveIt Visual Tools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, 
      move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  RCLCPP_INFO(logger, "Step 4: Setting up visualization functions...");

  // Test visual tools connection
  RCLCPP_INFO(logger, "Testing MoveItVisualTools connection...");
  moveit_visual_tools.trigger();
  
  // Give some time for RViz to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  RCLCPP_INFO(logger, "Visual tools should be connected now.");

  // Closure to make the draw_title function
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = []() {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };

  // Enhanced prompt function with logging and backup mechanism
  auto const prompt = [&moveit_visual_tools, &logger](auto text) {
    RCLCPP_INFO(logger, "PROMPT: %s", text);
    RCLCPP_INFO(logger, "Waiting for user input in RViz...");
    RCLCPP_INFO(logger, "If RViz prompt doesn't work, press Ctrl+C to exit or check RViz console");
    
    // Set a timeout for the prompt to avoid infinite hanging
    RCLCPP_INFO(logger, "Starting RViz prompt...");
    
    try {
      moveit_visual_tools.prompt(text);
      RCLCPP_INFO(logger, "User input received! Continuing...");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "Prompt failed with exception: %s", e.what());
      RCLCPP_ERROR(logger, "Falling back to terminal input...");
      std::cout << "\nPress Enter to continue (RViz prompt failed): ";
      std::cin.get();
    }
  };

  // ============================================
  // TCP (TOOL CENTER POINT) CONFIGURATION
  // ============================================
  
  RCLCPP_INFO(logger, "Step 5: Configuring TCP offset...");
  
  // TCP offset: 5cm in each direction from tool0 (units in meters)
  // Fixed: Proper Eigen conversion for Humble
  const Eigen::Isometry3d T_tool0_tcp = 
        Eigen::Isometry3d(Eigen::Translation3d(0.05, 0.05, 0.05));
  
  RCLCPP_INFO(logger, "TCP offset from tool0: [%.3f, %.3f, %.3f] meters",
              T_tool0_tcp.translation().x(), 
              T_tool0_tcp.translation().y(), 
              T_tool0_tcp.translation().z());

  // ============================================
  // DEMONSTRATION - Standard MoveIt 2 TCP Pattern
  // ============================================
  
  RCLCPP_INFO(logger, "Step 6: Starting demo visualization...");
  
  draw_title("MoveIt 2 TCP Offset Demo");
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 7: First prompt - Press 'next' to start...");
  prompt("Press 'next' to start the TCP offset demo");

  RCLCPP_INFO(logger, "Step 8: Setting start state...");
  // Set starting pose to current state
  move_group_interface.setStartStateToCurrentState();

  // ============================================
  // STEP 1: Visualize current TCP position
  // ============================================
  
  RCLCPP_INFO(logger, "Step 9: Getting current TCP pose...");
  auto current_tcp_pose = getCurrentTcpPose(move_group_interface);
  
  RCLCPP_INFO(logger, "Current TCP position: [%.3f, %.3f, %.3f]",
              current_tcp_pose.position.x,
              current_tcp_pose.position.y,
              current_tcp_pose.position.z);
  
  RCLCPP_INFO(logger, "Step 10: Visualizing current TCP...");
  // Visualize current TCP
  moveit_visual_tools.publishSphere(current_tcp_pose, rviz_visual_tools::BLUE, 0.015, "current_tcp");
  moveit_visual_tools.publishAxis(current_tcp_pose, 0.05, 0.01, "current_tcp_axes");
  moveit_visual_tools.publishText(current_tcp_pose, "Current TCP", rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 11: Second prompt - Press 'next' to plan...");
  prompt("Press 'next' to plan motion to target TCP pose");

  // ============================================
  // STEP 2: Define target TCP pose and plan
  // ============================================
  
  RCLCPP_INFO(logger, "Step 12: Defining target TCP pose...");
  
  // Define a target pose for the TCP (camera tip)
  geometry_msgs::msg::Pose target_tcp_pose;
  target_tcp_pose.orientation.w = 1.0;
  target_tcp_pose.position.x = 0.2;
  target_tcp_pose.position.y = 0.2;
  target_tcp_pose.position.z = 0.5;

  RCLCPP_INFO(logger, "Target TCP position: [%.3f, %.3f, %.3f]",
              target_tcp_pose.position.x,
              target_tcp_pose.position.y,
              target_tcp_pose.position.z);

  RCLCPP_INFO(logger, "Step 13: Converting TCP target to tool0 target...");
  // Convert TCP target to tool0 target using standard MoveIt 2 pattern
  auto target_tool0_pose = tool0PoseFromTcpPose(target_tcp_pose);

  RCLCPP_INFO(logger, "Tool0 target position: [%.3f, %.3f, %.3f]",
              target_tool0_pose.position.x,
              target_tool0_pose.position.y,
              target_tool0_pose.position.z);

  RCLCPP_INFO(logger, "Step 14: Setting pose target for tool0...");
  // Set the target for tool0 (not TCP) - this is what MoveIt solves IK for
  move_group_interface.setPoseTarget(target_tool0_pose, "tool0");

  RCLCPP_INFO(logger, "Step 15: Starting motion planning...");
  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const planning_success = move_group_interface.plan(plan);

  RCLCPP_INFO(logger, "Step 16: Planning completed with result: %s", 
              (planning_success == moveit::core::MoveItErrorCode::SUCCESS) ? "SUCCESS" : "FAILED");

  // Fixed: Use correct MoveItErrorCode namespace for Humble
  if (planning_success == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Planning successful! TCP will reach target pose.");
    
    // ============================================
    // STEP 3: Visualize target and trajectory
    // ============================================
    
    RCLCPP_INFO(logger, "Step 17: Visualizing target TCP pose...");
    
    // Visualize target TCP pose
    moveit_visual_tools.publishAxis(target_tcp_pose, rviz_visual_tools::LARGE);
    moveit_visual_tools.publishSphere(target_tcp_pose, rviz_visual_tools::GREEN, 0.02, "target_tcp");
    moveit_visual_tools.publishText(target_tcp_pose, "Target TCP", rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    
    // ============================================
    // KEY FEATURE: Visualize TCP trajectory manually for Humble compatibility
    // ============================================
    
    RCLCPP_INFO(logger, "Step 18: Drawing tool0 trajectory...");
    
    // First, draw the standard tool0 trajectory
    moveit_visual_tools.publishTrajectoryLine(
        plan.trajectory_,
        move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator"),
        rviz_visual_tools::YELLOW);  // Yellow for tool0 path
    
    RCLCPP_INFO(logger, "Step 19: Processing trajectory for TCP visualization...");
    
    // Now manually draw the TCP trajectory using trajectory points
    // Convert trajectory to RobotTrajectory for processing
    robot_trajectory::RobotTrajectory robot_traj(move_group_interface.getRobotModel());
    robot_traj.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), plan.trajectory_);
    
    RCLCPP_INFO(logger, "Trajectory has %zu waypoints", robot_traj.getWayPointCount());
    
    RCLCPP_INFO(logger, "Step 20: Drawing TCP trajectory points...");
    
    // Draw TCP points along the trajectory
    for (size_t i = 0; i < robot_traj.getWayPointCount(); i += 3) {  // Every 3rd point to reduce clutter
      const auto& waypoint = robot_traj.getWayPoint(i);
      const Eigen::Isometry3d& tool0_pose = waypoint.getGlobalLinkTransform("tool0");
      
      // Calculate TCP pose for this waypoint
      Eigen::Isometry3d tcp_pose = tool0_pose * T_tool0_tcp;
      
      // Convert to geometry_msgs::Pose for visualization
      geometry_msgs::msg::Pose tcp_pose_msg = tf2::toMsg(tcp_pose);
      
      // Draw small sphere at TCP position - Fixed: Use correct parameters for Humble
      moveit_visual_tools.publishSphere(tcp_pose_msg, rviz_visual_tools::CYAN, rviz_visual_tools::SMALL, "tcp_trajectory");
    }
    
    RCLCPP_INFO(logger, "Step 21: Drawing TCP trajectory line...");
    
    // Draw line connecting TCP points for better visualization
    std::vector<geometry_msgs::msg::Point> tcp_points;
    for (size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
      const auto& waypoint = robot_traj.getWayPoint(i);
      const Eigen::Isometry3d& tool0_pose = waypoint.getGlobalLinkTransform("tool0");
      Eigen::Isometry3d tcp_pose = tool0_pose * T_tool0_tcp;
      
      geometry_msgs::msg::Point p;
      p.x = tcp_pose.translation().x();
      p.y = tcp_pose.translation().y();
      p.z = tcp_pose.translation().z();
      tcp_points.push_back(p);
    }
    
    // Publish TCP trajectory line
    moveit_visual_tools.publishPath(tcp_points, rviz_visual_tools::CYAN, 0.004, "tcp_path");
    
    RCLCPP_INFO(logger, "Step 22: Triggering visualization update...");
    moveit_visual_tools.trigger();
    
    RCLCPP_INFO(logger, "Step 23: Third prompt - Press 'next' to execute...");
    prompt("Press 'next' to execute the motion");
    
    // ============================================
    // STEP 4: Execute motion
    // ============================================
    
    RCLCPP_INFO(logger, "Step 24: Updating title for execution...");
    draw_title("Executing Motion");
    moveit_visual_tools.trigger();
    
    RCLCPP_INFO(logger, "Step 25: Executing planned motion...");
    auto const execute_success = move_group_interface.execute(plan);
    
    RCLCPP_INFO(logger, "Step 26: Execution completed with result: %s", 
                (execute_success == moveit::core::MoveItErrorCode::SUCCESS) ? "SUCCESS" : "FAILED");
    
    // Fixed: Use correct MoveItErrorCode namespace for Humble
    if (execute_success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Motion executed successfully!");
      
      RCLCPP_INFO(logger, "Step 27: Getting final TCP pose...");
      // Visualize final TCP position
      auto final_tcp_pose = getCurrentTcpPose(move_group_interface);
      
      RCLCPP_INFO(logger, "Final TCP position: [%.3f, %.3f, %.3f]",
                  final_tcp_pose.position.x,
                  final_tcp_pose.position.y,
                  final_tcp_pose.position.z);
      
      RCLCPP_INFO(logger, "Step 28: Visualizing final TCP position...");
      moveit_visual_tools.publishSphere(final_tcp_pose, rviz_visual_tools::ORANGE, 0.02, "final_tcp");
      moveit_visual_tools.publishText(final_tcp_pose, "Final TCP", rviz_visual_tools::ORANGE, rviz_visual_tools::MEDIUM);
      moveit_visual_tools.trigger();
      
      draw_title("Motion Complete - TCP at Target");
      moveit_visual_tools.trigger();
      
      RCLCPP_INFO(logger, "Step 29: Calculating positioning accuracy...");
      // Calculate and display accuracy
      Eigen::Isometry3d target_transform, final_transform;
      tf2::fromMsg(target_tcp_pose, target_transform);
      tf2::fromMsg(final_tcp_pose, final_transform);
      
      double position_error = (target_transform.translation() - final_transform.translation()).norm();
      RCLCPP_INFO(logger, "TCP positioning accuracy: %.4f meters", position_error);
      
    } else {
      RCLCPP_ERROR(logger, "Motion execution failed!");
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  RCLCPP_INFO(logger, "Step 30: Final prompt - Press 'next' to finish...");
  prompt("Press 'next' to finish demo");
  
  RCLCPP_INFO(logger, "Step 31: Demo complete!");
  draw_title("TCP Demo Complete");
  moveit_visual_tools.trigger();

  RCLCPP_INFO(logger, "=== MoveIt TCP Demo Finished ===");
  
  // Proper shutdown following MoveIt2 documentation
  RCLCPP_INFO(logger, "Shutting down...");
  rclcpp::shutdown();  // This will cause the spin function in the thread to return
  spinner.join();  // Join the thread before exiting
  
  return 0;
}