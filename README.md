Hoping to add all of these into a single launch file in order to simplify the process. For the current moment this is how the script should be launched

// run this command in one terminal
// this will launch the robot with mocked hardware in RVIZ
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false

// run this command in a second terminal 
// Load MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true

// run this command in a 3rd terminal
// Run the path planning executable (this is the main executable that will also control the logic for the rest of the system)
ros2 launch ur3_bottle_sorter bottle_sorter.launch.py ur_type:=ur3

NOTES:
Need to add
- setting positions via the gui and necessary ros2 nodes
- reading bottle positions
- design the test setup (where the bottle are placed/held, bins to dump the bottles, how are these placed/fixed to the table/lab equipment
