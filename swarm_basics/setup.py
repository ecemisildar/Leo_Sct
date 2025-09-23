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
        ('share/' + package_name + '/launch', ['launch/spawn_multi_robots.launch.py']),
        ('share/' + package_name + '/launch', ['launch/leo_gz.launch.py']),
        ('share/' + package_name + '/config', ['config/supervisor.yaml']),
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
            'robot_circle_controller = swarm_basics.robot_circle_controller:main',
        ],
    },
)
