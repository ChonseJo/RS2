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
	// get the sim variable from launch command
	bool simulation_mode = false;
	node->get_parameter("simulation", simulation_mode);
	int runCount_ = 0;
	
	// values in meters
	double pickup_x = -0.05;
	double pickup_y = 0.22;
	double pickup_z = 0.216;
	double pickup_z_approach = 0.35;

	double bin_clear_z = 0.4;
	double b1_x = -0.2;
	double b1_y = 0.3;
	double b2_x = -0.2;
	double b2_y = 0.15;
	double b3_x = -0.2;
	double b3_y = 0.0;

	if (!simulation_mode) {
		RCLCPP_INFO(logger, "Testing Servos...");
		rclcpp::sleep_for(std::chrono::milliseconds(5000)); 
		servo_control->open_servos({0,0,0,0,0,0});
		rclcpp::sleep_for(std::chrono::milliseconds(3000)); 
		servo_control->open_servos({1,1,1,1,1,1});
	}
	else{
		RCLCPP_INFO(logger, "!!!!!!!!!! LAUNCHED IN SIMULATION MODE !!!!!!!!!!");
	}

	if(path_planner->getRunState() == false){
		RCLCPP_INFO(logger, "Waiting on Run Command...");
	}

	while(rclcpp::ok()){
		// Run Command
		// auto ee_pose = move_group_interface.getCurrentPose();
		// 	RCLCPP_INFO(logger, "Current Position:\n x = %f, y = %f, z = %f",
		// 		ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z);

		if(path_planner->getRunState() == false && simulation_mode){
			runCount_ = 0;
		}
		
		if(path_planner->getRunState()){
			
			path_planner->setJointGoal(60, -100, 50, -30, -95, -30); // camera detect position

			std::vector<bool> bottles;
			if(simulation_mode){
				if(runCount_ == 0){
					bottles = {true, true, true}; // C, F, S
				}
				else if(runCount_ == 1){
					bottles = {false, false, true}; // C, F, S
				}
				else{
					bottles = {false, false, false}; // C, F, S
				}
				runCount_++;
				RCLCPP_INFO(logger, "Simulated bottle detection.");
			} else {
				bottles = servo_control->requestBottlePositions();
				RCLCPP_INFO(logger, "Bottles Detected");
			}

			bool all_false = std::all_of(bottles.begin(), bottles.end(), [](bool v) { return !v; });
			if(all_false){ 
				path_planner->resetRunState();
				RCLCPP_INFO(logger, "Sequence Ended: No Bottles Detected");
				continue;
			}
			
			// way points
			path_planner->setJointGoal(57.09, -106.79, 58.73, -42.02, -89.86, 326.33); // flat position after bottle detect
			path_planner->setCartPose(pickup_x, pickup_y, pickup_z_approach);

			// pick up bottles
			path_planner->setCartPose(pickup_x, pickup_y, pickup_z);
			rclcpp::sleep_for(std::chrono::milliseconds(1000)); 
			if (!simulation_mode) {servo_control->close_servos();}
			rclcpp::sleep_for(std::chrono::milliseconds(3000)); 
			path_planner->setCartPose(pickup_x, pickup_y, bin_clear_z);
			

			// *** go to bin position *** 	
			if(bottles[0]){
				path_planner->setCartPose(b1_x, b1_y, bin_clear_z);
				RCLCPP_INFO(logger, "Dropping C");
				if (!simulation_mode) {servo_control->openBottleTypes("C");}
				rclcpp::sleep_for(std::chrono::milliseconds(5000));
			}
			if(bottles[1]){
				path_planner->setCartPose(b2_x, b2_y, bin_clear_z);
				RCLCPP_INFO(logger, "Dropping F");
				if (!simulation_mode) {servo_control->openBottleTypes("F");}
				rclcpp::sleep_for(std::chrono::milliseconds(5000));  
			}
			if(bottles[2]){
				path_planner->setCartPose(b3_x, b3_y, bin_clear_z);
				RCLCPP_INFO(logger, "Dropping S");
				if (!simulation_mode) {servo_control->openBottleTypes("S");}
				rclcpp::sleep_for(std::chrono::milliseconds(5000));  
			}

			// servo_control->open_servos({1,1,1,1,1,1});
			// rclcpp::sleep_for(std::chrono::milliseconds(5000));  

			// go home
			path_planner->setJointGoal(60, -100, 50, -30, -95, -30); // camera detect position
			rclcpp::sleep_for(std::chrono::milliseconds(2000));  
		}
	}

	// Shutdown ROS
	while(rclcpp::ok()){
		spinner.join();
	}

	rclcpp::shutdown();
	return 0;
}