from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
  # 获取URDF文件的路径
  pkg_share = get_package_share_directory('sealien_ctrlpilot_controller')
  urdf_file = os.path.join(pkg_share, 'urdf', 'auv.urdf.xml')
  
  # 读取URDF文件内容
  with open(urdf_file, 'r') as infp:
    robot_desc = infp.read()
  # 创建robot_state_publisher节点，将URDF内容传递给robot_description参数
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_desc}]
  )

  return LaunchDescription([
    launch_ros.actions.Node(
      package='sealien_ctrlpilot_controller',
      executable='msg_adapter_node',
      name='msg_adapter_node',         #name 要跟 executable一致，否则参数无法正常传输
      output='screen',
      parameters=[os.path.join(get_package_share_directory("sealien_ctrlpilot_controller"), 'config', 'msgs_adapter.yaml')],
      # remappings=[
      #     ('/sealien_mavros/imu_raw', '/ImuNavStatus'),
      #     ('/sealien_mavros/depthFinder', '/DepthStatus'),
      #     ('/sealien_mavros/heightStatus', '/SonarAltimeterStatus'),
      # ]
    ),
    robot_state_publisher_node,
])
