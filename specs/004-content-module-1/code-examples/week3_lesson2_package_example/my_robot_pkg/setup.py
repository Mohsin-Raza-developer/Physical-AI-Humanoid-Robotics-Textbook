from setuptools import find_packages, setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@example.com',
    description='Example ROS 2 package demonstrating basic package structure',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_robot_pkg.talker_node:main',
        ],
    },
)
