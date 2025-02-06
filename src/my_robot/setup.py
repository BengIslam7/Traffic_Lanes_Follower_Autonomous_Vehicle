from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='med-islam',
    maintainer_email='devmlpentBMI66@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "camera_s = my_robot.camera_subs:main",
        "camera_p = my_robot.camera_pub:main",
        "control_node = my_robot.control:main",
        "camera_talker = my_robot.camera_publisher:main",
        "camera_listener = my_robot.camera_subscriber:main"
        ],
    },
)
