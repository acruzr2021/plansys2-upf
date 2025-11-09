from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_file = LaunchConfiguration('model_file')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value='src/ros2_planning_system/'
        'plansys2_domain_expert/test/pddl/domain_simple.pddl',
        description='PDDL Model file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    # Specify the actions
    domain_expert_cmd = Node(
        package='domain_expert_node_upf',
        executable='domain_expert_node_upf',
        name='domain_expert_upf',
        namespace=namespace,
        output='screen',
        parameters=[
          {'model_file': model_file}, params_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(domain_expert_cmd)

    return ld
