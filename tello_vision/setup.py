from setuptools import setup
import os
from ament_index_python.packages import get_package_share_path


package_name = 'tello_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wallnuts',
    maintainer_email='wallnuts@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = tello_vision.aruco_detector_node:main',
            'tello_opencv = tello_vision.tello_opencv_node:main'
        ],
    },
)
