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


	if(path_planner->getRunState() == false){
		RCLCPP_INFO(logger, "Waiting on Run Command...");
	}

	double pickup_x = -0.05;
	double pickup_y = 0.22;
	double pickup_z = 0.214;
	double pickup_z_approach = 0.35;

	double bin_clear_z = 0.4;
	double b1_x = 0.2;
	double b1_y = 0.3;
	double b2_x = 0.2;
	double b2_y = 0.15;
	double b3_x = 0.2;
	double b3_y = 0.0;

	rclcpp::sleep_for(std::chrono::milliseconds(10000)); 
	servo_control->open_servos({1,1,1,1,1,1});
	rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
	servo_control->open_servos({0,0,0,0,0,0});

	while(rclcpp::ok()){
		// Run Command
		auto ee_pose = move_group_interface.getCurrentPose();
			RCLCPP_INFO(logger, "Current Position:\n x = %f, y = %f, z = %f",
				ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z);
		

		if(path_planner->getRunState()){

			path_planner->setJointGoal(60, -100, 50, -30, -95, -30); // starting position

			path_planner->setCartPose(pickup_x, pickup_y, pickup_z_approach);

			// pick up bottles
			path_planner->setCartPose(pickup_x, pickup_y, pickup_z);
			servo_control->close_servos();
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
			path_planner->setCartPose(pickup_x, pickup_y, pickup_z_approach);
			
			// *** go to bin position *** 		
			path_planner->setCartPose(b1_x, b1_y, bin_clear_z);
			servo_control->open_servos({1,1,1,1,1,1});
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

			path_planner->setCartPose(b2_x, b2_y, bin_clear_z);
			servo_control->open_servos({0,0,0,1,1,1});
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
			
			path_planner->setCartPose(b3_x, b3_y, bin_clear_z);
			servo_control->open_servos({0,0,0,0,0,0});
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

			// go home
			path_planner->setCartPose(b2_x, b2_y, pickup_z_approach);
			path_planner->setJointGoal(60, -100, 50, -30, -95, -30); // starting position

			// path_planner->setCartPose(pickup_x, pickup_y, pickup_z_approach);

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