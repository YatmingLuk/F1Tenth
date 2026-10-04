from setuptools import find_packages, setup

package_name = 'car_control'

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
    maintainer='chanky',
    maintainer_email='chanky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'pure_pursuit_tf = car_control.pure_pursuit_tf_ackermann:main',
           'waypoint_logger_tf = car_control.waypoint_logger_tf:main',
           'drive_to_vesc_bridge = car_control.drive_to_vesc_bridge:main',
        ],
    },
)
