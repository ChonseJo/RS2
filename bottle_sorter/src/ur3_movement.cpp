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

using namespace std::chrono_literals;

class BottleSorter : public rclcpp::Node
{
public:
  BottleSorter() : Node("bottle_sorter") 
  {    
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "tool0");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(1s, std::bind(&BottleSorter::getCurrentPose, this));

  }

  ~BottleSorter(){
  }

  void setMoveGroupPointer(moveit::planning_interface::MoveGroupInterface* ptr){
    move_group_interface_ = ptr;

    // Set the planning time and tolerance
    move_group_interface_->setPlanningTime(10.0);
    move_group_interface_->setGoalTolerance(0.1);
    move_group_interface_->setStartStateToCurrentState();

    moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);

    const moveit::core::JointModelGroup* joint_model_group_ =
    move_group_interface_->getCurrentState()->getJointModelGroup("ur_manipulator");
  }

  void setJointPose(double x, double y, double z, double roll, double pitch, double yaw)
  {
    // getCurrentState and getCurrentPose dont work???
    moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    // start_state.setFromIK(joint_model_group_, target_pose);
    // move_group_interface_->setStartState(start_state);
    move_group_interface_->setPoseTarget(target_pose);

    // Plan the motion
    // move_group_interface_->setPlanningTime(10.0); // default is 5 seconds
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

  void setCartPose(double x, double y, double z, double roll, double pitch, double yaw){
    // Get the current pose to define the start point for Cartesian motion
    std::unique_lock<std::mutex> lock(pose_mutex_);
    geometry_msgs::msg::Pose current_pose = current_pose_;
    lock.unlock();
    
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

  void cartesianIncrement(double x=0, double y=0, double z=0){
    // Get the current pose to define the start point for Cartesian motion
    std::unique_lock<std::mutex> lock(pose_mutex_);
    geometry_msgs::msg::Pose current_pose = current_pose_;
    lock.unlock();
    
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
    const double eef_step = 0.001;
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

  void getCurrentPose()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "base";

    geometry_msgs::msg::TransformStamped transform_stamped;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
      transform_stamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "Current Transform:");
      RCLCPP_INFO(this->get_logger(), "Position: x = %.2f, y = %.2f, z = %.2f", 
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z);

      // Update current_pose_ with the transform
      std::unique_lock<std::mutex> lock(pose_mutex_);
      current_pose_.position.x = transform_stamped.transform.translation.x;
      current_pose_.position.y = transform_stamped.transform.translation.y;
      current_pose_.position.z = transform_stamped.transform.translation.z;
      current_pose_.orientation = transform_stamped.transform.rotation;
      // lock.unlock();

    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
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

private:
  moveit::planning_interface::MoveGroupInterface* move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  const moveit::core::JointModelGroup* joint_model_group_;

  geometry_msgs::msg::Pose current_pose_;

  std::mutex pose_mutex_;  // Mutex to protect the shared variable (current_pose_)

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};