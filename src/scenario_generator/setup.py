import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scenario_generator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maritime Collision Avoidance Team',
    maintainer_email='user@todo.todo',
    description='场景生成器模块 - 自动生成各种船舶相遇场景',
    license='Apache-2.0',
    tests_require=['pytest', 'hypothesis'],
    entry_points={
        'console_scripts': [
            # 场景生成器命令行工具（后续实现）
        ],
    },
)
