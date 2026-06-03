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
        default_value=os.path.join(
            get_package_share_directory('plansys2_domain_expert'),
            'pddl', 'domain_simple.pddl'
        ),
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
        #prefix=f"bash -c 'source {os.environ['HOME']}/.pixi/envs/default/bin/activate && exec $0 $@'",
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

    executor_cmd = Node(
        package='plansys2_executor',
        executable='executor_node',
        name='executor',
        namespace=namespace,
        output='screen',
        parameters=[{
            'default_action_bt_xml_filename': os.path.join(
                get_package_share_directory('plansys2_executor'),
                'behavior_trees', 'plansys2_action_bt.xml'),
            'default_start_action_bt_xml_filename': os.path.join(
                get_package_share_directory('plansys2_executor'),
                'behavior_trees', 'plansys2_start_action_bt.xml'),
            'default_end_action_bt_xml_filename': os.path.join(
                get_package_share_directory('plansys2_executor'),
                'behavior_trees', 'plansys2_end_action_bt.xml'),
            'bt_builder_plugin': 'STNBTBuilder'
        }]
    )

    lifecycle_manager_cmd = Node(
        package='plansys2_lifecycle_manager',
        executable='lifecycle_manager_node',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'node_names': [
                'domain_expert',
                'problem_expert',
                'planner',
                'executor'
            ]
        }]
    )


    ld = LaunchDescription()

    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_problem_file_cmd)
    ld.add_action(declare_namespace_cmd)

    ld.add_action(domain_expert_cmd)
    ld.add_action(problem_expert_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(executor_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld