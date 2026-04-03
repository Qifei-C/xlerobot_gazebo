import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("xlerobot_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    backend = LaunchConfiguration("backend").perform(context).lower()
    official_use_degrees = LaunchConfiguration("official_use_degrees").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    if backend not in ("omni", "planar"):
        raise RuntimeError(f"Unsupported backend '{backend}'. Expected 'omni' or 'planar'.")

    gz_args = f"-r {'-s ' if headless else ''}{world_path}"

    urdf_name = "xlerobot.urdf.xacro" if backend == "omni" else "xlerobot_planar.urdf.xacro"
    controllers_file = (
        os.path.join(pkg_share, "config", "controllers.yaml")
        if backend == "omni"
        else os.path.join(pkg_share, "config", "controllers_planar.yaml")
    )
    base_controller_name = (
        "omni_wheel_velocity_controller" if backend == "omni" else "ideal_base_velocity_controller"
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                os.path.join(pkg_share, "urdf", urdf_name),
                " ",
                "controllers_file:=",
                controllers_file,
            ]
        ),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    if backend == "omni":
        command_bridge = Node(
            package="xlerobot_gazebo",
            executable="omni_twist_to_wheels.py",
            output="screen",
            parameters=[
                os.path.join(pkg_share, "config", "base_geometry.yaml"),
                os.path.join(pkg_share, "config", "base_envelope_real.yaml"),
                {
                    "input_topic": "/base/cmd_vel",
                    "output_topic": "/omni_wheel_velocity_controller/commands",
                    "limited_topic": "/base/cmd_vel_limited",
                }
            ],
        )

        odom_bridge = Node(
            package="xlerobot_gazebo",
            executable="omni_wheel_odometry.py",
            output="screen",
            parameters=[
                os.path.join(pkg_share, "config", "base_geometry.yaml"),
                {
                    "joint_states_topic": "/joint_states",
                    "odom_topic": "/odom",
                    "odom_frame_id": "odom",
                    "base_frame_id": "base_link",
                    "publish_tf": True,
                }
            ],
        )
    else:
        command_bridge = Node(
            package="xlerobot_gazebo",
            executable="twist_to_planar_cmd.py",
            output="screen",
            parameters=[
                os.path.join(pkg_share, "config", "planner_test_envelope.yaml"),
                {
                    "input_topic": "/base/cmd_vel",
                    "output_topic": "/ideal_base_velocity_controller/commands",
                    "limited_topic": "/base/cmd_vel_limited",
                }
            ],
        )
        odom_bridge = Node(
            package="xlerobot_gazebo",
            executable="planar_joint_odometry.py",
            output="screen",
            parameters=[
                {
                    "joint_states_topic": "/joint_states",
                    "odom_topic": "/odom",
                    "odom_frame_id": "odom",
                    "base_frame_id": "base_link",
                    "publish_tf": True,
                }
            ],
        )

    official_base_adapter = Node(
        package="xlerobot_gazebo",
        executable="xlerobot_official_base_adapter.py",
        output="screen",
        parameters=[
            {
                "input_action_topic": "/xlerobot/action_dict",
                "output_cmd_topic": "/base/cmd_vel",
                "input_odom_topic": "/odom",
                "output_observation_topic": "/xlerobot/observation_dict",
                "command_frame_id": "base_link",
                "use_degrees": official_use_degrees,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "xlerobot",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.10",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    left_arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "left_arm_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    left_gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "left_gripper_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            base_controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    right_arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "right_arm_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    head_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "head_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    right_gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "right_gripper_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.path.dirname(pkg_share),
        ),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        command_bridge,
        odom_bridge,
        official_base_adapter,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[omni_base_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=omni_base_controller_spawner,
                on_exit=[left_arm_position_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_arm_position_controller_spawner,
                on_exit=[left_gripper_position_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_gripper_position_controller_spawner,
                on_exit=[right_arm_position_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_arm_position_controller_spawner,
                on_exit=[right_gripper_position_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_gripper_position_controller_spawner,
                on_exit=[head_position_controller_spawner],
            )
        ),
    ]


def generate_launch_description():
    demo_pkg_share = get_package_share_directory("demo_bot_gazebo")
    default_world = os.path.join(demo_pkg_share, "worlds", "demo_world.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("backend", default_value="omni"),
            DeclareLaunchArgument("official_use_degrees", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
