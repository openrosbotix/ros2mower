import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2mower_map_provider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'example'), glob(os.path.join('example', '*.[yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'plugins'), glob('ros2mower_map_provider/mapPlugins/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Weber',
    maintainer_email='info@weberpatrick.de',
    description='ROS2Mower Map Server, provides Mow areas and their keepout zines',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_provider = ros2mower_map_provider.provider_node:main'
        ],
    },
)
