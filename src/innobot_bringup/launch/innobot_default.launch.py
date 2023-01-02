from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="Start robot with gripper attached to it.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slowdown", default_value="3.0", description="Slowdown factor of the RRbot."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="arm_joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_controller",
            default_value="gripper_position_controller",
            description="Gripper controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_calibration",
            default_value="false",
            description="Start calibrattion server if it is required.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_pnp",
            default_value="false",
            description="Start pnp server if it is required.",
        )
    )
    
    # Initialize Arguments
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_gripper = LaunchConfiguration("use_gripper")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    slowdown = LaunchConfiguration("slowdown")
    robot_controller = LaunchConfiguration("robot_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")
    use_calibration = LaunchConfiguration("use_calibration")
    use_pnp = LaunchConfiguration("use_pnp")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/innobot_base.launch.py"]),
        launch_arguments={
            "controllers_file": "ros_controllers.yaml",
            "description_file": "innobot.urdf.xacro",
            "prefix": prefix,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "slowdown": slowdown,
            "robot_controller": robot_controller,
            "gripper_controller": gripper_controller,
            "use_gripper": use_gripper,
            "use_calibration": use_calibration,
            "use_pnp": use_pnp,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [base_launch])