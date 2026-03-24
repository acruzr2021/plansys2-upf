from setuptools import setup, find_packages

package_name = 'plansys2_upf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/plansys2_upf/config', ['config/planner.yaml']),
    ],
    install_requires=[
        'setuptools',
        # 'plansys2_msgs',
        # 'plansys2_domain_expert',
        # 'plansys2_popf_plan_solver',
        'unified-planning>=1.3.0',
        'pyperplan'
    ],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Alba Cruz',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        # Verifica que estas rutas sean CORRECTAS
        'domain_upf_node = plansys2_upf.domain_expert_node_upf.domain_upf_node:main',
        'problem_upf_node = plansys2_upf.problem_expert_node_upf.problem_upf_node:main',
        'planner_upf_node = plansys2_upf.planner_upf.planner_upf_node:main'
    ],
},
)
