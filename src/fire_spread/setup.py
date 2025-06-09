from glob import glob
from setuptools import find_packages, setup

package_name = 'fire_spread'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/fire_spread/launch', glob('launch/*.launch.py')),
        ('share/fire_spread', ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'your_detection_msgs',
        'numpy',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='abdul',
    maintainer_email='a.m.z.gadaborchev@student.tue.nl',
    description='Simulate fire spread and exploration logic in ROS 2 + Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'victim_rescue = fire_spread.victim_rescue_node:main',
            'spawn_pillar_fire = fire_spread.spawn_pillar_fire:main',
            'navigator = fire_spread.navigator:main',
            'detector = fire_spread.bottle_detector:main',
        ],
    },
)
