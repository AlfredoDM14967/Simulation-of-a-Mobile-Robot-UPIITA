from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import os
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("ttrobot_description"),"urdf","rover.urdf.xacro"),
        description="Ruta del archivo robot URDF"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "simple_velocity_controller",
                "simple_position_controller",
                "--controller-manager",
                "/controller_manager"
            ]
        )
    
    simple_controller_py = Node(
        package="ttrobot_controller",
        executable="simple_controller.py",
        parameters=[{"wheel_radius": 0.105,
                     "wheel_separation" : 1.015}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("ttrobot_controller"),
                "config",
                "ttrobot_controllers.yaml",
            ),
        ],
    )

    # RPLIDAR = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("sllidar_ros2"),
    #         "launch",
    #         "sllidar_a3_launch.py"
    #     ),
    # )

    ############
    motores = Node(
        package="nodosros2",
        executable="motores"
    )

    imu_driver = Node(
        package="rover_firmware",
        executable="imu_driver.py"
    )

    scan_sub = Node(
        package="nodosros2",
        executable="scan_sub"
    )

    gps_driver = Node(
        package="rover_firmware",
        executable="gps_driver.py"
    )

    arduino_control = Node(
        package="rover_firmware",
        executable="arduino.py"
    )

    filtro_kalman = Node(
        package="robot_localizacion",
        executable="filtro_kalman.py"
    )

    return LaunchDescription([
        model_arg,
        use_simple_controller_arg,
        motores,
        simple_controller_py,
        # RPLIDAR,
        scan_sub,
        gps_driver,
        imu_driver,
        arduino_control,
        filtro_kalman,
    ])