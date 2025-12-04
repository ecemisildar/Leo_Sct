from setuptools import find_packages, setup

package_name = 'leo_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/leo_real.launch.py']),
        ('share/' + package_name + '/config', ['config/sup_gpt.yaml']),
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
            'robot_supervisor_3_movements = leo_real.robot_supervisor_3_movements:main',
        ],
    },
)
