import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

  stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
  package_share = get_package_share_directory('dddnav_odom_3d')
  bag_path = os.path.join(package_share, 'bag_files', 'rosbag2_odom2d_imu')

  odom_3d_ros = Node(
          package="dddnav_odom_3d",
          executable="odom_3d",
          output="screen",
  )  

  rviz = Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('dddnav_odom_3d'), 'rviz', 'odom_3d_example_.rviz')]
  )  
  
  bag_player = ExecuteProcess(
      cmd=[
          "ros2",
          "bag",
          "play",
          "--loop",
          bag_path,
      ],
      output="screen",
  )

  ld = LaunchDescription()

  ld.add_action(stdout_colorized_envvar)
  ld.add_action(odom_3d_ros)
  ld.add_action(rviz)
  ld.add_action(TimerAction(period=1.0, actions=[bag_player]))

  return ld