from setuptools import setup, find_packages

package_name = 'domain_expert_node_upf'

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
    ],
    install_requires=[
        'setuptools',
        'plansys2_msgs',
        'plansys2_domain_expert',
        'plansys2_popf_plan_solver'
    ],
    zip_safe=True,
    maintainer='Alba Cruz',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'domain_upf_node = domain_expert_node_upf.domain_upf_node:main',
        ],
    },
)
