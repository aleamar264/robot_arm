import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


urdf_file = os.path.join(get_package_share_directory('bazu'), 'urdf', 'bazu.urdf')
model_name = 'bazu'
declare_urdf_path_cmd = DeclareLaunchArgument('urdf_path', default_value=urdf_file, description='Path to the URDF file')
declare_model_name_cmd = DeclareLaunchArgument('model_name', default_value=model_name, description='Name of the model')

spawn_entity_launch_file = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'spawn_entity_demo.launch.py')
spawn_entity_node_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(spawn_entity_launch_file),
    launch_arguments={
        'entity': LaunchConfiguration('model_name'),
        'x': '0',
        'y': '0',
        'z': '0',
        'file': LaunchConfiguration('urdf_path')
    }.items()
)

def generate_launch_description():
    return LaunchDescription([
        declare_urdf_path_cmd,
        declare_model_name_cmd,
        spawn_entity_node_cmd
    ])
