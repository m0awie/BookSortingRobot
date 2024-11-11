from setuptools import setup

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-contrib-python', 'pyrealsense2'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package for ArUco marker detection using RealSense camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_detection.aruco_node:main',
            'robot_node = aruco_detection.robot_node:main',
            'gripper_node = aruco_detection.gripper_node:main',
        ],
    },
)

