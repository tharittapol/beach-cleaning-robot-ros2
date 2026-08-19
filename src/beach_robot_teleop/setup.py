from setuptools import setup

package_name = 'beach_robot_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Joystick teleop for 4-wheel skid-steer base',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_4wd = beach_robot_teleop.teleop_4wd_node:main',
        ],
    },
)
