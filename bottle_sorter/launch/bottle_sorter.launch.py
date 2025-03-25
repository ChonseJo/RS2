from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

package_directory = get_package_share_directory('bottle_sorter')
config_dir = os.path.join(package_directory, 'config')

# Load the robot description (URDF)
urdf_file = os.path.join(config_dir, "ur3e.urdf")  # Update this with your URDF file
srdf_file = os.path.join(config_dir, "ur3e.srdf")  # Update this with your SRDF file

# Ensure the kinematics file exists
kinematics_file = os.path.join(config_dir, "kinematics.yaml")  # Update this with your kinematics file


# Define xacro mappings for the robot description file
launch_arguments = {
    "robot_ip": "xxx.yyy.zzz.www",
    "use_fake_hardware": "true",
}

# Load the robot configuration
moveit_config = (
    MoveItConfigsBuilder("ur3e", package_name="bottle_sorter")
    .robot_description(mappings=launch_arguments)
    .robot_description_kinematics(kinematics_file)
    # .robot_description_semantic(srdf_file)
    .planning_pipelines(pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"])
    .to_moveit_configs()
)

run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],
)

# Static TF
static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
)

# Publish TF
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],
)

def generate_launch_description():
    

    return LaunchDescription([
        run_move_group_node,
        robot_state_publisher,
        static_tf
    ]
)