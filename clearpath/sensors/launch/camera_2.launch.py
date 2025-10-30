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
    node_camera_2_gz_bridge = Node(
        name='camera_2_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a201_0000/sensors/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': '/root/clearpath/sensors/config/camera_2.yaml'
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_2_static_tf = Node(
        name='camera_2_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='a201_0000',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'camera_2_link'
                ,
                '--child-frame-id'
                ,
                'a201_0000/robot/base_link/camera_2'
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

    node_camera_2_gz_image_bridge = Node(
        name='camera_2_gz_image_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        namespace='a201_0000/sensors/',
        output='screen',
        arguments=
            [
                '/a201_0000/sensors/camera_2/image'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/a201_0000/sensors/camera_2/image'
                    ,
                    '/a201_0000/sensors/camera_2/_/image_raw'
                )
                ,
                (
                    '/a201_0000/sensors/camera_2/image/compressed'
                    ,
                    '/a201_0000/sensors/camera_2/_/compressed'
                )
                ,
                (
                    '/a201_0000/sensors/camera_2/image/compressedDepth'
                    ,
                    '/a201_0000/sensors/camera_2/_/compressedDepth'
                )
                ,
                (
                    '/a201_0000/sensors/camera_2/image/theora'
                    ,
                    '/a201_0000/sensors/camera_2/_/theora'
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

    node_camera_2_gz_cmd_bridge = Node(
        name='camera_2_gz_cmd_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a201_0000/sensors/',
        output='screen',
        arguments=
            [
                '/a201_0000/sensors/camera_2/cmd_pan_vel@std_msgs/msg/Float64]gz.msgs.Double'
                ,
                '/a201_0000/sensors/camera_2/cmd_tilt_vel@std_msgs/msg/Float64]gz.msgs.Double'
                ,
                '/a201_0000/sensors/camera_2/pan_joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
                ,
                '/a201_0000/sensors/camera_2/tilt_joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/a201_0000/sensors/camera_2/pan_joint_state'
                    ,
                    '/a201_0000/platform/joint_states'
                )
                ,
                (
                    '/a201_0000/sensors/camera_2/tilt_joint_state'
                    ,
                    '/a201_0000/platform/joint_states'
                )
                ,
                (
                    '/a201_0000/sensors/camera_2/cmd_pan_vel'
                    ,
                    '/a201_0000/sensors/camera_2/_/cmd_pan_vel'
                )
                ,
                (
                    '/a201_0000/sensors/camera_2/cmd_tilt_vel'
                    ,
                    '/a201_0000/sensors/camera_2/_/cmd_tilt_vel'
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

    node_ptz_action_server_node = Node(
        name='ptz_action_server_node',
        executable='ptz_controller_node',
        package='clearpath_generator_gz',
        namespace='/a201_0000/sensors/camera_2',
        output='screen',
        remappings=
            [
                (
                    'image_in'
                    ,
                    '/a201_0000/sensors/camera_2/_/image_raw'
                )
                ,
                (
                    'image_out'
                    ,
                    '/a201_0000/sensors/camera_2/color/image'
                )
                ,
                (
                    'cmd/velocity'
                    ,
                    '/a201_0000/sensors/camera_2/cmd/velocity'
                )
                ,
                (
                    'joint_states'
                    ,
                    '/a201_0000/platform/joint_states'
                )
                ,
                (
                    'cmd_pan_vel'
                    ,
                    '/a201_0000/sensors/camera_2/_/cmd_pan_vel'
                )
                ,
                (
                    'cmd_tilt_vel'
                    ,
                    '/a201_0000/sensors/camera_2/_/cmd_tilt_vel'
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
                {
                    'camera_name': 'camera_2'
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_camera_2_gz_bridge)
    ld.add_action(node_camera_2_static_tf)
    ld.add_action(node_camera_2_gz_image_bridge)
    ld.add_action(node_camera_2_gz_cmd_bridge)
    ld.add_action(node_ptz_action_server_node)
    return ld
