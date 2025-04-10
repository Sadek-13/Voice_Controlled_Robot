from launch import LaunchDescription
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    ur3robot_description_dir = get_package_share_directory("ur3robot_description")
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(ur3robot_description_dir, "urdf", "ur3robot.urdf.xacro"), description="Absolute path to robot urdf file")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value=[str(Path(ur3robot_description_dir).parent.resolve()
                   )
                ]
    )

    ros_distro = os.environ["ROS_DISTRO"]
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description, "use_sim_time":True}]
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        '/opt/ros/humble/share/ros_gz_sim/launch/gz_sim.launch.py'
    ),
    launch_arguments=[
        ("gz_args", f"-v 4 -r empty.sdf {physics_engine}")
    ]
    )

    gz_spwan_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "ur3robot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock/gz.msgs.Clock"
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spwan_entity,
        gz_ros2_bridge
    ])