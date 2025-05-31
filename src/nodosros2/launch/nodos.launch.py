from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    Camara_py = Node(
        package="nodosros2",
        executable="Camara"
    )

    ConMan_py = Node(
        package="nodosros2",
        executable="Controller_manager"
    )

    GPS_py = Node(
        package="nodosros2",
        executable="GPS"
    )

    LIDAR_py = Node(
        package="nodosros2",
        executable="LIDAR"
    )

    Motores_py = Node(
        package="nodosros2",
        executable="Motores"
    )

    Rover_py = Node(
        package="nodosros2",
        executable="rover_IHMremota"
    )

    STM_py = Node(
        package="nodosros2",
        executable="STM"
    )

    return LaunchDescription([
        Camara_py,
        ConMan_py,
        GPS_py,
        LIDAR_py,
        Motores_py,
        Rover_py,
        STM_py
    ])