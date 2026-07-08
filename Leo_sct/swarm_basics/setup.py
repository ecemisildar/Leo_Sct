from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', ['config/cylinder_positions.json']),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', ['worlds/random_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ecem',
    maintainer_email='ecem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_supervisor = swarm_basics.robot_supervisor:main',
            'random_walk_cbf_controller = swarm_basics.random_walk_cbf_controller:main',
            'robot_proximity_warner = swarm_basics.robot_proximity_warner:main',
            'robot_id_warning_relay = swarm_basics.robot_id_warning_relay:main',
            'coverage_counter = swarm_basics.coverage_counter:main',
            'bump_counter = swarm_basics.bump_counter:main',
            'plot_bump_reasons = swarm_basics.plot_bump_reasons:main',
        ],
    },
)
