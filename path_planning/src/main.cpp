#include "path_planning.cpp"
#include <thread>

int main(int argc, char * argv[])
{
	// ======================================================================
	// ROS2 INIT
	// ======================================================================
	rclcpp::init(argc, argv); // init ros2
	auto const node = std::make_shared<rclcpp::Node>( 
		"bottle_sorter",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
	); // creates the main node for running background moveit functionality

	// Create a ROS logger
	auto const logger = rclcpp::get_logger("bottle_sorter");

	// ======================================================================
	// PATH PLANNER INIT
	// ======================================================================

	// create path_planner object
	auto path_planner = std::make_shared<PathPlanning>();

    // ======================================================================
    // SPINNING THE NODE AND EXECUTOR
    // ======================================================================
    rclcpp::executors::SingleThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(node);
    executor.add_node(path_planner);  // Add the PathPlanning node to the executor

    // Spin the executor in a separate thread
    std::thread spinner([&executor]() {executor.spin();});

	// Create the MoveIt MoveGroup Interface
	using moveit::planning_interface::MoveGroupInterface;
	auto move_group_interface = MoveGroupInterface(path_planner, "ur_manipulator");
	path_planner->setMoveGroupPointer(&move_group_interface);
	path_planner->setScene(); // set the planning scene

    // ======================================================================
	// project code starts here
    // ======================================================================
	auto ee_pose = move_group_interface.getCurrentPose();
    // Log the current end-effector pose
    RCLCPP_INFO(logger, "End-effector pose: %f %f %f",
		ee_pose.pose.position.x,
      	ee_pose.pose.position.y,
      	ee_pose.pose.position.z
    );

	double roll = 0.0;
	double pitch = 3.14;
	double yaw = 0.0;

	double x = 0.05;
	double y = 0.05;
	double z = 0.05;

	path_planner->setJointGoal(0, -90, 0, -90, 0, 0); // home position
	path_planner->setJointGoal(90, -90, 90, -90, -90, 0); // starting position
	// path_planner->setJointPose(0.3, 0.0, 0.35, roll, pitch, yaw);

	// // used to make sure the robot is facing straight down
	// ee_pose = move_group_interface.getCurrentPose();
	// path_planner->setCartPose(	ee_pose.pose.position.x,
	// 							ee_pose.pose.position.y,
	// 							ee_pose.pose.position.z,
	// 							roll, pitch, yaw);

	path_planner->cartesianIncrement(x, 0, 0);
	path_planner->cartesianIncrement(0, y, 0);
	path_planner->cartesianIncrement(-x, 0, 0);
	path_planner->cartesianIncrement(0, -y, 0);

	// Shutdown ROS
	while(rclcpp::ok()){
		spinner.join();
	}

	rclcpp::shutdown();
	return 0;
}

// ======================================================================
// SUDO CODE
//  ======================================================================
// go to picture point
// take picture
// bottle detection and ML models and whatever 

// move to bin 1
// drop bottles for bin 1

// move to bin 2
// drop bottles for bin 2

// move to bin 3
// drop bottles for bin 3

// go back to photo point