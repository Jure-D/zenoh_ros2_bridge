
import os

from ament_index_python.packages import get_package_share_directory
import launch

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg


def generate_launch_description():

    optitrack_params_file_path = os.path.join(get_package_share_directory(
      'zenoh_ros2_bridge'), 'config', 'optitrack_driver_params.yaml')
    
    zenoh_ros2_bridge_params_file_path = os.path.join(get_package_share_directory(
      'zenoh_ros2_bridge'), 'config', 'zenoh_ros2_bridge_params.yaml')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # print('')
    # print('params_file_path: ', params_file_path)
    # print('')

    driver_node = LifecycleNode(
        name='mocap4r2_optitrack_driver_node',
        namespace=LaunchConfiguration('namespace'),
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        output='screen',
        parameters=[optitrack_params_file_path],
    )

    # Make the driver node take the 'configure' transition
    driver_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matchers.matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the driver node take the 'activate' transition
    driver_activate_trans_event = EmitEvent(
       event = ChangeState(
           lifecycle_node_matcher = launch.events.matchers.matches_action(driver_node),
           transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    zenoh_ros2_bridge_node = Node(
        name="zenoh_ros2_bridge",
        namespace="",
        package="zenoh_ros2_bridge",
        executable="zenoh_ros2_bridge",
        output="screen",
        parameters=[zenoh_ros2_bridge_params_file_path]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(driver_node)
    ld.add_action(driver_configure_trans_event)
    ld.add_action(driver_activate_trans_event)

    ld.add_action(zenoh_ros2_bridge_node)

    return ld