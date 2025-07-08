import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    # Get shared package path
    mycobot280_pkg_share = get_package_share_directory("mycobot280")

    moveit_config = (
        MoveItConfigsBuilder("mycobot280", package_name="mycobot280")
        .robot_description(file_path=os.path.join(mycobot280_pkg_share, "urdf", "robot.urdf.xacro"))
        .robot_description_semantic(file_path="config/mycobot280.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")  # ✅ <--- added!
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config = os.path.join(mycobot280_pkg_share, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("mycobot280"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node,
            rviz_node,
            controller
        ]
    )
