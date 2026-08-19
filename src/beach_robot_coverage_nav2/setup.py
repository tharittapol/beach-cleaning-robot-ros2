from setuptools import setup
from glob import glob

package_name = 'beach_robot_coverage_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.pgm')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Boustrophedon coverage planner + Nav2 keepout/boundary mask bringup.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coverage_follow_waypoints = beach_robot_coverage_nav2.coverage_follow_waypoints:main',
            'generate_keepout_mask = beach_robot_coverage_nav2.generate_keepout_mask:main',
            'obstacle_detector = beach_robot_coverage_nav2.obstacle_detector:main',
            'zed_obstacle_stop = beach_robot_coverage_nav2.zed_obstacle_stop_node:main',
            'footprint_viz = beach_robot_coverage_nav2.footprint_viz:main',
        ],
    },
)
