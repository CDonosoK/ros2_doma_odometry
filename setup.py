from setuptools import find_packages, setup

package_name = 'ros2_doma_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/sensors_launch.launch.py']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdonoso',
    maintainer_email='clemente.donosok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ros2_doma_odometry.sensors.camera_node:main',
            'gps_node = ros2_doma_odometry.sensors.gps_node:main',
            'imu_node = ros2_doma_odometry.sensors.imu_node:main',
        ],
    },
)