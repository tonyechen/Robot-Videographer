from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo2motor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_photographer',
    maintainer_email='todo@todo.com',
    description='Pan a camera joint to track YOLO person detections via JointTrajectory commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo2motor = yolo2motor.yolo2motor:main',
        ],
    },
)
