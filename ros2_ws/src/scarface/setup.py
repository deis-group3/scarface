from setuptools import find_packages, setup

package_name = 'scarface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scarface.launch.py']),
        ('share/' + package_name + '/config', ['config/scarface_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DEIS Group 3',
    maintainer_email='deis-group3@example.com',
    description='ROS2 Jazzy adapter for the Scarface robot fish',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scarface_node = scarface.scarface_node:main',
        ],
    },
)
