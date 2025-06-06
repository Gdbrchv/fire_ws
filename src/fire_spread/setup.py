from setuptools import find_packages, setup

package_name = 'fire_spread'

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
    maintainer='abdul',
    maintainer_email='a.m.z.gadaborchev@student.tue.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_spread_node = fire_spread.fire_spread_node:main',
	    'explorer_node = fire_spread.explorer_node:main',
        ],
    },
)
