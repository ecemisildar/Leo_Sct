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
        ('share/' + package_name + '/config', ['config/sup_gpt.yaml']),
        ('share/' + package_name + '/config', ['config/cylinder_positions.json']),
        ('share/' + package_name + '/worlds', ['worlds/random_world.sdf']),
        ('share/' + package_name + '/worlds', ['worlds/aruco_4x4_0.png']),
        ('share/' + package_name + '/worlds/textures', ['worlds/textures/aruco_4x4_0.png']),
        ('share/' + package_name + '/models/aruco_marker_0', [
            'models/aruco_marker_0/model.config',
            'models/aruco_marker_0/model.sdf',
        ]),
        ('share/' + package_name + '/models/aruco_marker_0/materials/scripts', [
            'models/aruco_marker_0/materials/scripts/aruco_marker.material',
        ]),
        ('share/' + package_name + '/models/aruco_marker_0/materials/textures', [
            'models/aruco_marker_0/materials/textures/aruco_4x4_0.png',
        ]),
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
            'coverage_counter = swarm_basics.coverage_counter:main',
            'bump_counter = swarm_basics.bump_counter:main',
        ],
    },
)
