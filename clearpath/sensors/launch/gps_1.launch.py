from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='')

    prefix = LaunchConfiguration('prefix')

    # Nodes
    node_gps_1_gz_bridge = Node(
        name='gps_1_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a201_0000/sensors/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/root/clearpath/sensors/config/gps_1.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    node_gps_1_static_tf = Node(
        name='gps_1_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='a201_0000',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'gps_1_link'
                ,
                '--child-frame-id'
                ,
                'a201_0000/robot/base_link/gps_1'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
                (
                    '/tf_static'
                    ,
                    'tf_static'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_gps_1_gz_bridge)
    ld.add_action(node_gps_1_static_tf)
    return ld
