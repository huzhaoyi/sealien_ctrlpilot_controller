
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='sealien_ctrlpilot_controller',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("sealien_ctrlpilot_controller"), 'config', 'default.yaml')],
            # remappings=[
            #     # ('/sealien_mavros/imu', '/ImuNavStatus'),
            #     # ('/sealien_mavros/depthFinder', '/DepthStatus'),
            #     # ('/sealien_mavros/heightStatus', '/SonarAltimeterStatus'),
            # ]
           ),
])


# import launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, LogInfo
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#      # 拼接文件路径
#     config_file_path = os.path.join(get_package_share_directory('sealien_ctrlpilot_controller'), 'config', 'default.yaml')

#     print(f"Current config_file_path: {config_file_path}")  # 打印当前目录
    
#     # 声明参数文件路径的启动参数
#     return LaunchDescription([
        
#         # 启动节点并加载参数文件
#         Node(
#             package='sealien_ctrlpilot_controller',  # 你的包名
#             executable='controller',  # 你的可执行文件名
#             name='controller',  # 节点名称
#             output='screen',
#             parameters=[config_file_path],  # 加载指定的 YAML 配置文件
#         ),
        
#         # 打印日志确认节点启动
#         LogInfo(
#             condition=launch.conditions.LaunchConfigurationEquals('config_file', ''),
#             msg="Starting with default parameter file"
#         )
#     ])

