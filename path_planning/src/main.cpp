#include "path_planning.cpp"
#include "servo_control.cpp"
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
	// INIT INCLUDED FILES
	// ======================================================================
	// create path_planner object
	auto path_planner = std::make_shared<PathPlanning>();
	auto servo_control = std::make_shared<ServoControl>();



    // ======================================================================
    // SPINNING THE NODE AND EXECUTOR
    // ======================================================================
    rclcpp::executors::SingleThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(node);
    executor.add_node(servo_control);
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

	path_planner->init_services();

	if(path_planner->getRunState() == false){
		RCLCPP_INFO(logger, "Waiting on Run Command...");
	}

	double b1_x = 0.2;
	double b1_y = 0.3;
	double b2_x = 0.2;
	double b2_y = 0.15;
	double b3_x = 0.2;
	double b3_y = 0.0;

	while(rclcpp::ok()){
		// Run Command
		if(path_planner->getRunState()){

			path_planner->setJointGoal(90, -90, 90, -90, -90, 0); // starting position

			auto ee_pose = move_group_interface.getCurrentPose();
			auto home_x = ee_pose.pose.position.x;
			auto home_y = ee_pose.pose.position.y;
			auto home_z = ee_pose.pose.position.z;

			// pick up bottles
			path_planner->setCartPose(home_x, home_y, home_z-0.05);
			servo_control->close_servos();
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
			path_planner->setCartPose(home_x, home_y, home_z);
			
			// *** go to bin position *** 		
			path_planner->setCartPose(b1_x, b1_y, home_z);
			servo_control->open_servos({1,1,1,1,1,1});
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

			path_planner->setCartPose(b2_x, b2_y, home_z);
			servo_control->open_servos({0,0,0,1,1,1});
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
			
			path_planner->setCartPose(b3_x, b3_y, home_z);
			servo_control->open_servos({0,0,0,0,0,0});
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

			// go home
			path_planner->setJointGoal(90, -90, 90, -90, -90, 0); // starting position
			path_planner->resetRunState();
			RCLCPP_INFO(logger, "Sequence completed. Bottles have been sorted");
		}
	}

	// Shutdown ROS
	while(rclcpp::ok()){
		spinner.join();
	}

	rclcpp::shutdown();
	return 0;
}