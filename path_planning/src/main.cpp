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
	// path_planner->setJointGoal(0, -90, 0, -90, 0, 0); // home position

	path_planner->init_services();

	if(path_planner->getRunState() == false){
		RCLCPP_INFO(logger, "Waiting on Run Command...");
	}

	while(rclcpp::ok()){
		// Run Command
		if(path_planner->getRunState()){

			path_planner->setJointGoal(90, -90, 90, -90, -90, 0); // starting position

			auto ee_pose = move_group_interface.getCurrentPose();
			path_planner->setCartPose(ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z-0.05);
			// pickup bottles

			path_planner->setCartPose(ee_pose.pose.position.x, ee_pose.pose.position.y,ee_pose.pose.position.z);
			
			// *** go to bin position *** 		
			path_planner->setCartPose(0.2,0.3,ee_pose.pose.position.z);
			// drop bottles
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

			path_planner->setCartPose(0.2,0.15,ee_pose.pose.position.z);
			// drop bottles
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
			
			path_planner->setCartPose(0.2,0,ee_pose.pose.position.z);
			// drop bottles
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

			// go home
			path_planner->setJointGoal(90, -90, 90, -90, -90, 0); // starting position

			// path_planner->setCartPose(ee_pose.position.x, ee_pose.position.y, ee_pose.position.z-0.2);
			path_planner->resetRunState();
		}
	}

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