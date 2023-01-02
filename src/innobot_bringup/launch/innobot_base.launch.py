
import os, yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def generate_launch_description():
    
    kinematics_file = get_package_file('innobot_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('innobot_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file('innobot_moveit_config', 'config/controllers.yaml')
    joint_limits_file = get_package_file('innobot_moveit_config','config/joint_limits.yaml')
    
    moveit_cpp_calibration_yaml_file_name = (
        get_package_share_directory("innobot_moveit_config") + "/config/moveit_cpp.yaml"
    )

    moveit_cpp_pnp_yaml_file_name = (
        get_package_share_directory("innobot_moveit_config") + "/config/moveit_cpp_pnp.yaml"
    )
    
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    joint_limits_config = load_yaml(joint_limits_file)

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_pipeline_config["ompl"].update(ompl_config)

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="innobot_moveit_config",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="innobot_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            description="URDF/XACRO description file with the robot.",
        )
    )
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
            "use_sim",
            default_value="false",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
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
            "slowdown", default_value="3.0", description="Slowdown factor of the Inobot."
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
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="Start robot with gripper attached.",
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
            description="Start pnp server, must be false if use_calibration is set to true.",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    slowdown = LaunchConfiguration("slowdown")
    robot_controller = LaunchConfiguration("robot_controller")
    gripper_controller = LaunchConfiguration("gripper_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    use_gripper = LaunchConfiguration("use_gripper")
    use_calibration = LaunchConfiguration("use_calibration")
    use_pnp = LaunchConfiguration("use_pnp")

    

    if(use_gripper == "true"):
        srdf_file = get_package_file('innobot_moveit_config', 'config/innobot.srdf')
    else:
        srdf_file = get_package_file('innobot_moveit_config', 'config/innobot_alone.srdf')

    robot_description_semantic = load_file(srdf_file)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "slowdown:=",
            slowdown,
            " ",
            "use_gripper:=",
            use_gripper,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            controllers_file,
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "innobot.rviz"]
    )

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }

    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

       
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                # 'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
            },
            ompl_planning_pipeline_config,
            robot_description,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            robot_controllers
            ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[
            {
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            },
            robot_description,
        ],
        condition=IfCondition(start_rviz),
    )    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    # only start gripper if use_gripper set to true (use_gripper set to true by default)

    gripper_as = Node(
            package='innobot_gripper', 
            executable='action_server', 
            output='screen',
            condition=IfCondition(use_gripper))

    gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[gripper_controller,"--controller-manager","/controller_manager"],
            output="screen",
            condition=IfCondition(use_gripper))

    # launch innobot calibration server if required

    arm_pnp_as = Node(
            name='innobot_node',
            package='innobot_core',
            executable='innobot_node',
            output='screen',
            parameters=[
                {
                    'base_frame': 'world',
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    'robot_description_planning' : joint_limits_config,
                    'planning_pipelines': ['ompl'],
                    # 'ompl': ompl_config
                },
                # moveit_cpp_config,
                ompl_planning_pipeline_config,
                robot_description,
                moveit_cpp_pnp_yaml_file_name,
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
            condition=IfCondition(use_pnp)
        )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        move_group_node,
        gripper_controller_spawner,
        gripper_as,
        arm_pnp_as,        

    ]

    return LaunchDescription(declared_arguments + nodes)