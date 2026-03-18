import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'collision_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maritime Collision Avoidance Team',
    maintainer_email='user@todo.todo',
    description='避碰决策模块 - 基于COLREGS规则的智能避碰算法',
    license='Apache-2.0',
    tests_require=['pytest', 'hypothesis'],
    entry_points={
        'console_scripts': [
            'ca_node = collision_avoidance.collision_avoidance_node:main',
        ],
    },
)
