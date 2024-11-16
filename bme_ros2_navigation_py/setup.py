from setuptools import find_packages, setup

package_name = 'bme_ros2_navigation_py'

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
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_republisher = bme_ros2_navigation_py.map_republisher:main',
            'send_initialpose = bme_ros2_navigation_py.send_initialpose:main',
            'slam_toolbox_load_map = bme_ros2_navigation_py.slam_toolbox_load_map:main',
            'follow_waypoints = bme_ros2_navigation_py.follow_waypoints:main',
        ],
    },
)
