import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('anritsu_driver'),
        'config',
        'anritsu_driver.yaml'
    )

    # Declare launch arguments
    nm = LaunchConfiguration('name')
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    nm_launch_arg = DeclareLaunchArgument(
        'name',
        default_value='anritsu_driver'
    )
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config
    )
    ld.add_action(nm_launch_arg)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Node parameters
    port = LaunchConfiguration('port')

    port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value="9001"
    )

    ld.add_action(port_launch_arg)

    # Create node launch description
    node = Node(
        package='anritsu_driver',
        executable='anritsu_driver_app',
        # prefix='gdb -ex run --args',
        # prefix='gdbserver localhost:8081',
        name=nm,
        namespace=ns,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[
            cf
        ]
    )

    ld.add_action(node)

    return ld
