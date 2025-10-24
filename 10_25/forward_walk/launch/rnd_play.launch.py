from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # topic_leg_state = DeclareLaunchArgument("TOPIC_WHEEL_STATE", default_value="/joint_states")
    # topic_leg_command = DeclareLaunchArgument("TOPIC_WHEEL_COMMAND", default_value="/position_controller/commands")
    # topic_imu = DeclareLaunchArgument("TOPIC_IMU", default_value="/imu")

    sthexa_control_launch = os.path.join( get_package_share_directory('rnd_control'), 'launch', 'rnd_control.launch.py')
    sthexa_gazebo_control = IncludeLaunchDescription( PythonLaunchDescriptionSource(sthexa_control_launch) )

    car_position_controller = Node( package='forward_walk',
                                     executable='main_node',
                                     name='position_controller_node',
                                     output='screen' )

    # return LaunchDescription([ topic_leg_state, topic_leg_command, topic_imu, sthexa_gazebo_control, car_position_controller ])
    return LaunchDescription([ sthexa_gazebo_control, car_position_controller ])
