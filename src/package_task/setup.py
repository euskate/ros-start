from setuptools import find_packages, setup

package_name = 'package_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JangWon',
    maintainer_email='euskate07@gmail.com',
    description='ROS2 Package Task',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_node = package_task.task_node:main',
            'turtle_controller = package_task.turtle_controller:main',
            'turtle_spawn = package_task.turtle_spawn:main',
            'turtle_race = package_task.turtle_race:main',
            'turtle_patterns = package_task.turtle_patterns:main',
        ],
    },
)
