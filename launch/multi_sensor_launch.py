from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import OpaqueFunction


def generate_nodes(context, *args, **kwargs):
    num_float_sensors = int(context.launch_configurations['float'])
    num_temperature_sensors = int(context.launch_configurations['temperature'])

    nodes = []

    # Generate float sensor nodes
    for i in range(num_float_sensors):
        node = Node(
            package='xberry_test',
            executable='sensor_float',
            name=f'sensor_float_{i}',
            output='screen',
            parameters=[{'sensor_id': i}]
        )
        nodes.append(node)

    # Generate temperature sensor nodes
    for i in range(num_temperature_sensors):
        node = Node(
            package='xberry_test',
            executable='sensor_temperature',
            name=f'temperature_sensor_{i}',
            output='screen',
            parameters=[{'sensor_id': i}]
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    num_float_sensors = DeclareLaunchArgument(
        'float',
        default_value='3',  # Default to 3 float sensors
        description='Number of float sensor nodes to launch'
    )

    num_temperature_sensors = DeclareLaunchArgument(
        'temperature',
        default_value='2',  # Default to 2 temperature sensors
        description='Number of temperature sensor nodes to launch'
    )

    return LaunchDescription([
        num_float_sensors,
        num_temperature_sensors,
        OpaqueFunction(function=generate_nodes)
    ])
