#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h> //used for low-level tracjevtory structures
#include <tf2_eigen/tf2_eigen.hpp>  

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <iostream> // Added for terminal input
#include <chrono>   // Added for sleep

// Compute tool0 pose for a desired TCP pose since this is what Moveit2 uses to plan
geometry_msgs::msg::Pose tool0PoseFromTcpPose(const geometry_msgs::msg::Pose& tcp_in_world,
                                               const Eigen::Isometry3d& T_tool0_tcp)
{
  Eigen::Isometry3d T_world_tcp;
  tf2::fromMsg(tcp_in_world, T_world_tcp);

  // Standard robotics transform: T_world_tool0 = T_world_tcp * (T_tool0_tcp)^-1
  Eigen::Isometry3d T_world_tool0 = T_world_tcp * T_tool0_tcp.inverse();
  return tf2::toMsg(T_world_tool0);
}

/** Get the current TCP pose in world frame */
geometry_msgs::msg::Pose getCurrentTcpPose(const moveit::planning_interface::MoveGroupInterface& move_group,
                                            const Eigen::Isometry3d& T_tool0_tcp)
{
  // Get current tool0 pose in world frame
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

  // spin up a SingleThreadedExecutor so MoveItVisualTools can interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(logger, "Step 2: Creating MoveGroup interface...");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set the end-effector link once since its not the true end effector in the udrf
  move_group_interface.setEndEffectorLink("tool0");

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
  
  // TCP offset: Configurable via ROS parameters (units in meters)
  double tcp_offset_x = node->get_parameter_or("tcp_offset.x", 0.05);
  double tcp_offset_y = node->get_parameter_or("tcp_offset.y", 0.05);
  double tcp_offset_z = node->get_parameter_or("tcp_offset.z", 0.05);
  
  // from 3 independent points -> 3d vector -> homogeneous matrix
  const Eigen::Isometry3d T_tool0_tcp = 
        Eigen::Isometry3d(Eigen::Translation3d(tcp_offset_x, tcp_offset_y, tcp_offset_z));
  
  RCLCPP_INFO(logger, "TCP offset from tool0: [%.3f, %.3f, %.3f] meters",
              T_tool0_tcp.translation().x(), 
              T_tool0_tcp.translation().y(), 
              T_tool0_tcp.translation().z());

  // ============================================
  // FOCUS POINT DEFINITION SYSTEM
  // ============================================
  
  RCLCPP_INFO(logger, "Step 7: Defining focus point...");
  
  // Assume user has manually positioned robot with TCP above and facing the focus point
  draw_title("Focus Point Definition");
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 8: Getting current TCP position for focus point calculation...");
  prompt("Position the robot manually so the TCP is directly above and facing the desired focus point, then press 'next'");
  
  // Capture the initial TCP position (this is where user positioned it)
  auto initial_tcp_pose = getCurrentTcpPose(move_group_interface, T_tool0_tcp);
  
  RCLCPP_INFO(logger, "Initial TCP position captured: [%.3f, %.3f, %.3f]",
              initial_tcp_pose.position.x,
              initial_tcp_pose.position.y,
              initial_tcp_pose.position.z);
  
  // Visualize the initial TCP position
  moveit_visual_tools.publishSphere(initial_tcp_pose, rviz_visual_tools::BLUE, 0.015, "initial_tcp");
  moveit_visual_tools.publishAxis(initial_tcp_pose, 0.05, 0.01, "initial_tcp_axes");
  moveit_visual_tools.publishText(initial_tcp_pose, "Initial TCP", rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 9: Getting focus point distance from user...");
  
  // Get distance input from user with improved prompting
  double focus_offset = 0.0;
  prompt("Record the distance to the focal-point and when you are ready to input press 'next'");
  std::cout << "\nEnter focus-point offset along TCP's -Z axis (meters, e.g., 0.08 = 8cm): ";
  std::cin >> focus_offset;
  
  // Validate input
  if (focus_offset <= 0.0 || focus_offset > 1.0) {
    RCLCPP_WARN(logger, "Invalid distance input: %.3f. Using default of 0.08m", focus_offset);
    focus_offset = 0.08;
  }
  
  RCLCPP_INFO(logger, "Focus point offset: %.3f meters", focus_offset);
  
  // ============================================
  // PROPER COORDINATE FRAME TRANSFORMATION
  // ============================================
  
  // Convert TCP pose to Eigen transform for proper coordinate math
  Eigen::Isometry3d T_world_tcp;
  tf2::fromMsg(initial_tcp_pose, T_world_tcp);
  
  // Calculate focus point: translate +Z in the TCP frame (positive blue axis direction)
  // This correctly handles TCP orientation regardless of robot pose
  Eigen::Isometry3d T_world_focus = T_world_tcp * 
                                    Eigen::Translation3d(0, 0, focus_offset); 
  
  // Convert back to ROS message format for display
  geometry_msgs::msg::Pose focus_point = tf2::toMsg(T_world_focus);
  
  RCLCPP_INFO(logger, "Step 10: Focus point calculated using proper coordinate transforms:");
  RCLCPP_INFO(logger, "  TCP frame: Translate [0, 0, %.3f] (along TCP's +Z axis - blue axis)", focus_offset);
  RCLCPP_INFO(logger, "  World coordinates: [%.3f, %.3f, %.3f]",
              focus_point.position.x,
              focus_point.position.y,
              focus_point.position.z);
  
  // ============================================
  // FOCUS POINT VISUALIZATION
  // ============================================
  
  RCLCPP_INFO(logger, "Step 11: Visualizing focus point...");
  
  // Visualize the focus point
  moveit_visual_tools.publishSphere(focus_point, rviz_visual_tools::PURPLE, 0.02, "focus_point");
  moveit_visual_tools.publishAxis(focus_point, 0.06, 0.01, "focus_point_axes");
  moveit_visual_tools.publishText(focus_point, "FOCUS POINT", rviz_visual_tools::PURPLE, rviz_visual_tools::LARGE);
  
  // Enhanced visualization: Draw a line connecting TCP to focus point
  std::vector<geometry_msgs::msg::Point> focus_line;
  geometry_msgs::msg::Point tcp_point, focus_pt;
  tcp_point.x = initial_tcp_pose.position.x;
  tcp_point.y = initial_tcp_pose.position.y;
  tcp_point.z = initial_tcp_pose.position.z;
  focus_pt.x = focus_point.position.x;
  focus_pt.y = focus_point.position.y;
  focus_pt.z = focus_point.position.z;
  focus_line.push_back(tcp_point);
  focus_line.push_back(focus_pt);
  
  moveit_visual_tools.publishPath(focus_line, rviz_visual_tools::YELLOW, 0.005, "tcp_to_focus");
  
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 12: Focus point visualization complete!");
  RCLCPP_INFO(logger, "  - Purple sphere and axes: Focus point location");
  RCLCPP_INFO(logger, "  - Yellow line: TCP to focus point connection");
  RCLCPP_INFO(logger, "  - Blue sphere: Initial TCP position");
  draw_title("Focus Point Defined - Ready for Motion");
  moveit_visual_tools.trigger();
  
  prompt("Focus point has been defined and visualized. Press 'next' to continue with existing trajectory");

  // ============================================
  // DEMONSTRATION - Standard MoveIt 2 TCP Pattern
  // ============================================
  
  RCLCPP_INFO(logger, "Step 13: Starting demo visualization...");
  
  draw_title("MoveIt 2 TCP Offset Demo");
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 14: Setting start state...");
  // Set starting pose to current state
  move_group_interface.setStartStateToCurrentState();

  // ============================================
  // STEP 1: Visualize current TCP position
  // ============================================
  
  RCLCPP_INFO(logger, "Step 15: Getting current TCP pose...");
  auto current_tcp_pose = getCurrentTcpPose(move_group_interface, T_tool0_tcp);
  
  RCLCPP_INFO(logger, "Current TCP position: [%.3f, %.3f, %.3f]",
              current_tcp_pose.position.x,
              current_tcp_pose.position.y,
              current_tcp_pose.position.z);
  
  RCLCPP_INFO(logger, "Step 16: Visualizing current TCP...");
  // Visualize current TCP
  moveit_visual_tools.publishSphere(current_tcp_pose, rviz_visual_tools::BLUE, 0.015, "current_tcp");
  moveit_visual_tools.publishAxis(current_tcp_pose, 0.05, 0.01, "current_tcp_axes");
  moveit_visual_tools.publishText(current_tcp_pose, "Current TCP", rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
  moveit_visual_tools.trigger();
  
  RCLCPP_INFO(logger, "Step 17: Second prompt - Press 'next' to plan...");
  prompt("Press 'next' to plan motion to target TCP pose");

  // ============================================
  // STEP 2: Define target TCP pose and plan
  // ============================================
  
  RCLCPP_INFO(logger, "Step 18: Defining target TCP pose...");
  
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

  RCLCPP_INFO(logger, "Step 19: Converting TCP target to tool0 target...");
  // Convert TCP target to tool0 target using standard MoveIt 2 pattern
  auto target_tool0_pose = tool0PoseFromTcpPose(target_tcp_pose, T_tool0_tcp);

  RCLCPP_INFO(logger, "Tool0 target position: [%.3f, %.3f, %.3f]",
              target_tool0_pose.position.x,
              target_tool0_pose.position.y,
              target_tool0_pose.position.z);

  RCLCPP_INFO(logger, "Step 20: Setting pose target for tool0...");
  // Set the target for tool0 (not TCP) - this is what MoveIt solves IK for
  move_group_interface.setPoseTarget(target_tool0_pose);

  RCLCPP_INFO(logger, "Step 21: Starting motion planning...");
  // Plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const planning_success = move_group_interface.plan(plan);

  RCLCPP_INFO(logger, "Step 22: Planning completed with result: %s", 
              (planning_success == moveit::core::MoveItErrorCode::SUCCESS) ? "SUCCESS" : "FAILED");

  // Fixed: Use correct MoveItErrorCode namespace for Humble
  if (planning_success == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Planning successful! TCP will reach target pose.");
    
    // ============================================
    // STEP 3: Visualize target and trajectory
    // ============================================
    
    RCLCPP_INFO(logger, "Step 23: Visualizing target TCP pose...");
    
    // Visualize target TCP pose
    moveit_visual_tools.publishAxis(target_tcp_pose, rviz_visual_tools::LARGE);
    moveit_visual_tools.publishSphere(target_tcp_pose, rviz_visual_tools::GREEN, 0.02, "target_tcp");
    moveit_visual_tools.publishText(target_tcp_pose, "Target TCP", rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    
    // ============================================
    // KEY FEATURE: Visualize TCP trajectory manually for Humble compatibility
    // ============================================
    
    RCLCPP_INFO(logger, "Step 24: Drawing tool0 trajectory...");
    
    // First, draw the standard tool0 trajectory
    moveit_visual_tools.publishTrajectoryLine(
        plan.trajectory_,
        move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator"),
        rviz_visual_tools::YELLOW);  // Yellow for tool0 path
    
    RCLCPP_INFO(logger, "Step 25: Processing trajectory for TCP visualization...");
    
    // Now manually draw the TCP trajectory using trajectory points
    // Convert trajectory to RobotTrajectory for processing
    robot_trajectory::RobotTrajectory robot_traj(move_group_interface.getRobotModel());
    robot_traj.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), plan.trajectory_);
    
    RCLCPP_INFO(logger, "Trajectory has %zu waypoints", robot_traj.getWayPointCount());
    
    RCLCPP_INFO(logger, "Step 26: Drawing TCP trajectory points...");
    
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
    
    RCLCPP_INFO(logger, "Step 27: Drawing TCP trajectory line...");
    
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
    
    RCLCPP_INFO(logger, "Step 28: Triggering visualization update...");
    moveit_visual_tools.trigger();
    
    RCLCPP_INFO(logger, "Step 29: Third prompt - Press 'next' to execute...");
    prompt("Press 'next' to execute the motion");
    
    // ============================================
    // STEP 4: Execute motion using plan()+execute() \
    // ============================================
    
    RCLCPP_INFO(logger, "Step 30: Updating title for execution...");
    draw_title("Executing Motion");
    moveit_visual_tools.trigger();
    
    RCLCPP_INFO(logger, "Step 31: Executing planned motion...");
    // Use execute(plan) 
    auto const execute_success = move_group_interface.execute(plan);
    
    RCLCPP_INFO(logger, "Step 32: Execution completed with result: %s", 
                (execute_success == moveit::core::MoveItErrorCode::SUCCESS) ? "SUCCESS" : "FAILED");
    
    //  MoveItErrorCode 
    if (execute_success == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "Motion executed successfully!");
      
      RCLCPP_INFO(logger, "Step 33: Getting final TCP pose...");
      // Visualize final TCP position
      auto final_tcp_pose = getCurrentTcpPose(move_group_interface, T_tool0_tcp);
      
      RCLCPP_INFO(logger, "Final TCP position: [%.3f, %.3f, %.3f]",
                  final_tcp_pose.position.x,
                  final_tcp_pose.position.y,
                  final_tcp_pose.position.z);
      
      RCLCPP_INFO(logger, "Step 34: Visualizing final TCP position...");
      moveit_visual_tools.publishSphere(final_tcp_pose, rviz_visual_tools::ORANGE, 0.02, "final_tcp");
      moveit_visual_tools.publishText(final_tcp_pose, "Final TCP", rviz_visual_tools::ORANGE, rviz_visual_tools::MEDIUM);
      moveit_visual_tools.trigger();
      
      draw_title("Motion Complete - TCP at Target");
      moveit_visual_tools.trigger();
      
      RCLCPP_INFO(logger, "Step 35: Calculating positioning accuracy...");
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

  RCLCPP_INFO(logger, "Step 36: Final prompt - Press 'next' to finish...");
  prompt("Press 'next' to finish demo");
  
  RCLCPP_INFO(logger, "Step 37: Demo complete!");
  draw_title("TCP Demo Complete");
  moveit_visual_tools.trigger();

  RCLCPP_INFO(logger, "=== MoveIt TCP Demo Finished ===");
  
  // Proper shutdown following MoveIt2 documentation
  RCLCPP_INFO(logger, "Shutting down...");
  rclcpp::shutdown();  // This will cause the spin function in the thread to return
  spinner.join();  // Join the thread before exiting
  
  return 0;
}