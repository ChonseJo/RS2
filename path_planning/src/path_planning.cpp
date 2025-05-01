#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <thread>
#include <mutex>
#include <cmath> // for M_PI
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class PathPlanning : public rclcpp::Node
{
public:
  PathPlanning() : Node("path_planning"){   
    run_state_ = false;
  }

  ~PathPlanning(){
  }

  void setMoveGroupPointer(moveit::planning_interface::MoveGroupInterface* ptr){
    move_group_interface_ = ptr;

    // Set the planning time and tolerance
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setGoalTolerance(0.01);
    move_group_interface_->setStartStateToCurrentState();
    init_services();
  }

  double degToRad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  void setJointGoal(int j1, int j2, int j3, int j4, int j5, int j6){
    // Set the target joint positions
    std::vector<double> target_joint_positions = {degToRad(j1), degToRad(j2), degToRad(j3), 
                                                  degToRad(j4), degToRad(j5), degToRad(j6)};    

    move_group_interface_->setJointValueTarget(target_joint_positions);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // moveit_commander::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("MoveIt"), "Planning successful. Executing the plan...");
        
        // Execute the motion
        move_group_interface_->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("MoveIt"), "Planning failed.");
    }
    return;
  }

  void setJointPose(double x, double y, double z, double roll, double pitch, double yaw)
  {    
    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    
    // Set the target orientation (Quaternion) from roll, pitch, yaw
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    move_group_interface_->setPoseTarget(target_pose);

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success){
      RCLCPP_INFO(this->get_logger(), "Plan successful. Moving to target...");
      move_group_interface_->execute(plan);
    }
    else{
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }
  }

  void setCartPoseOrientation(double x, double y, double z, double roll, double pitch, double yaw){
    auto current_pose = move_group_interface_->getCurrentPose().pose;
    
    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    // Set the target orientation (Quaternion) from roll, pitch, yaw
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    // Set the pose target for MoveGroup
    move_group_interface_->setPoseTarget(target_pose);
    
    // Define waypoints for the Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);  // start at current pose
    waypoints.push_back(target_pose);   // move to target pose

    // Compute the Cartesian path (with a resolution of 1 cm, feel free to adjust it)
    const double eef_step = 0.01;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);

    RCLCPP_INFO(this->get_logger(), "Cartesian path computed with %.2f%% success", fraction * 100.0);

    // Execute the path if successful
    if (fraction > 0.9) {
        RCLCPP_INFO(this->get_logger(), "Executing Cartesian path...");
        move_group_interface_->execute(trajectory);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed with %.2f%% success", fraction * 100.0);
    }
  }

  void setCartPose(double x, double y, double z){
    auto current_pose = move_group_interface_->getCurrentPose().pose;
    
    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    // Set the target orientation (Quaternion)
    target_pose.orientation.x = current_pose.orientation.x;
    target_pose.orientation.y = current_pose.orientation.y;
    target_pose.orientation.z = current_pose.orientation.z;
    target_pose.orientation.w = current_pose.orientation.w;

    // Set the pose target for MoveGroup
    move_group_interface_->setPoseTarget(target_pose);
    
    // Define waypoints for the Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);  // start at current pose
    waypoints.push_back(target_pose);   // move to target pose

    // Compute the Cartesian path (with a resolution of 1 cm, feel free to adjust it)
    const double eef_step = 0.01;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);

    RCLCPP_INFO(this->get_logger(), "Cartesian path computed with %.2f%% success", fraction * 100.0);

    // Execute the path if successful
    if (fraction > 0.9) {
        RCLCPP_INFO(this->get_logger(), "Executing Cartesian path...");
        move_group_interface_->execute(trajectory);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed with %.2f%% success", fraction * 100.0);
    }
  }

  void cartesianIncrement(double x=0, double y=0, double z=0){
    auto current_pose = move_group_interface_->getCurrentPose().pose;
    
    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = current_pose.position.x + x;
    target_pose.position.y = current_pose.position.y + y;
    target_pose.position.z = current_pose.position.z + z;

    // Set the target orientation (Quaternion)
    target_pose.orientation.x = current_pose.orientation.x;
    target_pose.orientation.y = current_pose.orientation.y;
    target_pose.orientation.z = current_pose.orientation.z;
    target_pose.orientation.w = current_pose.orientation.w;

    // Set the pose target for MoveGroup
    move_group_interface_->setPoseTarget(target_pose);

    // Define waypoints for the Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);  // start at current pose
    waypoints.push_back(target_pose);   // move to target pose

    // Compute the Cartesian path (with a resolution of 1 cm, feel free to adjust it)
    const double eef_step = 0.01;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, 0.0, trajectory);

    RCLCPP_INFO(this->get_logger(), "Cartesian path computed with %.2f%% success", fraction * 100.0);

    // Execute the path if successful
    if (fraction > 0.9) {
        RCLCPP_INFO(this->get_logger(), "Executing Cartesian path...");
        move_group_interface_->execute(trajectory);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed with %.2f%% success", fraction * 100.0);
    }
  }

  void setScene()
  {
    // Create the collision object for the robot to avoid
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_->getPlanningFrame();
    collision_object.id = "table";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 5.0;
    primitive.dimensions[primitive.BOX_Y] = 5.0;
    primitive.dimensions[primitive.BOX_Z] = 1.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.51;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
  }

  void init_services(){
    toggle_run_service_ = this->create_service<std_srvs::srv::SetBool>(
      "toggle_run",
      std::bind(&PathPlanning::handle_toggle_run, this, std::placeholders::_1, std::placeholders::_2)
    );

    call_home_service_ = this->create_service<std_srvs::srv::SetBool>(
      "call_home",
      std::bind(&PathPlanning::handle_call_home, this, std::placeholders::_1, std::placeholders::_2)
    );


  }

  void handle_call_home(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    (void)request;
    response->success = true;
    response->message = "Calling Home Position";

    setJointGoal(0, -90, 0, -90, 0, 0);
  }

  void handle_toggle_run(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  { 
    (void)request;
    run_state_ = !run_state_;
    response->success = true;
    response->message = run_state_ ? "Run state enabled." : "Run state disabled.";
  }

  bool getRunState(){
    return run_state_;
  }

  void resetRunState(){
    run_state_ = false;
  }

private:
  moveit::planning_interface::MoveGroupInterface* move_group_interface_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_run_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr call_home_service_;
  bool run_state_;
};