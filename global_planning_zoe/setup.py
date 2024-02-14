import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'global_planning_zoe'
lib = 'global_planning_zoe/lib'

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
    maintainer='abbad',
    maintainer_email='abbad.aicha.manar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = global_planning_zoe.main_node:main',
            'astar_node = global_planning_zoe.astar_node:main',
            'openstreetmap_node = global_planning_zoe.openstreetmap_node:main',
        ],
    },
)
