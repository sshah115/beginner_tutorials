from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bag_record = LaunchConfiguration('bag_record')

    bag_record_arg = DeclareLaunchArgument(
        'bag_record',
        default_value='False'
    )
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker'
    )
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener'
    )
    server_node = Node(
        package='beginner_tutorials',
        executable='server'
    )
    bag_record_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([bag_record,' == True'])
        ),
        cmd=[[
            'cd ../result /bag_files && ros2 bag record /topic '
        ]],
        shell=True
    )

    return LaunchDescription([
        bag_record_arg,
        talker_node,
        listener_node,
        server_node,
        TimerAction(
            period=2.0,
            actions=[bag_record_conditioned],
        )
    ])
