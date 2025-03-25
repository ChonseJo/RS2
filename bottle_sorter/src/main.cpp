#include "ur3_movement.cpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto bottle_sorter = std::make_shared<BottleSorter>();
  
  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface move_group_interface_ (bottle_sorter, "ur_manipulator");
  bottle_sorter->setMoveGroupPointer(&move_group_interface_);
  
  bottle_sorter->setScene();

  // Start rclcpp::spin in a separate thread to handle timer callbacks and other async tasks
  std::thread spin_thread([&](){
    rclcpp::spin(bottle_sorter);  // Handle ROS callbacks in background
  });

  // Specify target XYZ and orientation (roll, pitch, yaw)

  moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState(10);

  double roll = 0.0;
  double pitch = 3.14;
  double yaw = 0.0;

  bottle_sorter->setJointPose(0.3, 0.0, 0.3, roll, pitch, yaw);

  bottle_sorter->cartesianIncrement(0.01, 0.0, 0.0);
  bottle_sorter->cartesianIncrement(0.0, 0.0, 0.01);
  bottle_sorter->cartesianIncrement(0.0, 0.0, -0.01);
  bottle_sorter->cartesianIncrement(0.01, 0.0, 0.0);
  
  while(rclcpp::ok()){
    spin_thread.join();    
  }
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}