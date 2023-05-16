import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

MODEL_NAME = 'bazu'
pkg_share = FindPackageShare(package='bazu').find('bazu')
default_urdf_model_path = os.path.join(pkg_share, 'urdf/bazu.urdf')

def generate_launch_description():

    default_empty_world_path = os.path.join(pkg_share, 'world/empty.world')
    # URDF model
    urdf_model = LaunchConfiguration('urdf_model')

    declare_model_name_cmd = DeclareLaunchArgument(
        'model_name', default_value=MODEL_NAME, description='Name of the model')
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    params = [{'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro ', urdf_model])}]

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= params)

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=params,
        arguments=[default_urdf_model_path])

    spawn_entity_launch_file = os.path.join(get_package_share_directory(
        'gazebo_ros'), 'launch', 'spawn_entity_demo.launch.py')
    spawn_entity_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity_launch_file),
        launch_arguments={
            'entity': LaunchConfiguration('model_name'),
            'x': '0',
            'y': '0',
            'z': '0',
            'file': LaunchConfiguration('urdf_model')
        }.items()
    )

    spawn_controller = Node(package = 'controller_manager',
                            executable = 'ros2_control_node',
                            name = 'controller_manager',
                            parameters=params,
                            output = 'screen')

    ld = LaunchDescription()
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(declare_model_name_cmd)
    ld.add_action(spawn_entity_node_cmd)
    ld.add_action(spawn_controller)

    return ld

