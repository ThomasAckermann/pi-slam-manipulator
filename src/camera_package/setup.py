from setuptools import setup
import os
from glob import glob


package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyslam'],
    zip_safe=True,
    maintainer='Thomas Ackermann',
    maintainer_email='ackermann.th@outlook.com',
    description='Camera package for Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_package.camera_publisher:main',
            'pyslam_camera_node = camera_package.pyslam_camera_node:main',
        ],
    },
)
