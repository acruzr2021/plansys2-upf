from setuptools import setup

package_name = 'domain_expert_node_upf'

setup(
    name=package_name,
    version='0.0.0',
    packages=['src', package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/resource', ['resource/domain_expert_node_upf']),
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
            'bumpgo = bt_bumpgo.bumpgo:main',
        ],
    },
)
