from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os


def generate_launch_description():

    package_name = 'plansys2_upf'

    pkg = get_package_share_directory(package_name)

    config = os.path.join(pkg, "config", "planner.yaml")

    model_file = LaunchConfiguration('model_file')
    problem_file = LaunchConfiguration('problem_file')
    namespace = LaunchConfiguration('namespace')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value='/home/alba/Documents/ws_tfg/src/ros2_planning_system/plansys2_popf_plan_solver/test/pddl/domain_simple.pddl',
        description='PDDL Model file'
    )

    declare_problem_file_cmd = DeclareLaunchArgument(
        'problem_file',
        default_value='',
        description='PDDL Problem file'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace'
    )

    domain_expert_cmd = Node(
        package=package_name,
        executable='domain_upf_node',
        name='domain_expert',
        namespace=namespace,
        output='screen',
        parameters=[
            config,
            {'model_file': model_file}
        ]
    )

    problem_expert_cmd = Node(
        package=package_name,
        executable='problem_upf_node',
        name='problem_expert',
        namespace=namespace,
        output='screen',
        parameters=[
            config,
            {
                'model_file': model_file,
                'problem_file': problem_file
            }
        ]
    )

    planner_cmd = Node(
        package=package_name,
        executable='planner_upf_node',
        name='planner',
        namespace=namespace,
        output='screen',
        parameters=[
            config,
            {
                'model_file': model_file,
                'problem_file': problem_file
            }
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_problem_file_cmd)
    ld.add_action(declare_namespace_cmd)

    ld.add_action(domain_expert_cmd)
    ld.add_action(problem_expert_cmd)
    ld.add_action(planner_cmd)

    return ld