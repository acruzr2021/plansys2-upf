from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'plansys2_upf'

    model_file = LaunchConfiguration('model_file')
    problem_file = LaunchConfiguration('problem_file')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value='src/ros2_planning_system/'
                      'plansys2_domain_expert/test/pddl/domain_simple.pddl',
        description='PDDL Model file'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Optional parameters file'
    )

    domain_expert_cmd = Node(
        package=package_name,
        executable='domain_upf_node',
        name='domain_expert_upf',
        namespace=namespace,
        output='screen',
        parameters=[
            {'model_file': model_file}
        ] + ([params_file] if params_file.perform({}) else []),
    )

    problem_expert_cmd = Node(
        package=package_name,
        executable='problem_upf_node',
        name='problem_expert_upf',
        namespace=namespace,
        output='screen',
        parameters=[
            {'model_file': model_file, 'problem_file': problem_file}
        ] + ([params_file] if params_file.perform({}) else []),
    )

    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(domain_expert_cmd)
    ld.add_action(problem_expert_cmd)

    return ld
