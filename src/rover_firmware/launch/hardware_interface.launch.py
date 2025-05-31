import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("ttrobot_description"),
                    "urdf",
                    "rover.urdf.xacro",
                )
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("rovercontroller"),
                "config",
                "ttrobot_controllers.yaml",
            ),
        ],
    )

    GPS_Nodo = Node(
        package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            parameters=[{
                'port': '/dev/ttyAMA0',
                'baud': 9600,
                'frame_id': 'gps_frame'
            }],
            output='screen',
            remappings=[
                ('/fix', '/gps/fix'),  # Publica en el tema /gps/fix
                ('/vel', '/gps/vel')   # Publica la velocidad en el tema /gps/vel
            ]
    )

    return LaunchDescription(
        [   
            robot_state_publisher_node,
            controller_manager,
            #GPS_Nodo
        ]
    )