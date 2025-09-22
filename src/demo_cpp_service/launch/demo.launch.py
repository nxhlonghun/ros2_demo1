import launch
import launch_ros

def generate_launch_description():
    #声明一个launch参数
    action_declare_arg_background_g=launch.actions.DeclareLaunchArgument('launch_arg_background',default_value="150")
    #把launch参数手动传给某个节点

    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[{'background_g':launch.substitutions.LaunchConfiguration('launch_arg_background',default="150")}],
        output='screen'
    )
    action_node_partol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='partol_client',
        output='screen'
    )
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        output='screen'
    )
    return launch.LaunchDescription([
        action_declare_arg_background_g,
        action_node_turtlesim_node,
        action_node_partol_client,
        action_node_turtle_control,
    ])