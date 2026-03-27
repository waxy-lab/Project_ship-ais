import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wenshan_pro'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config', 'scenarios'),
            glob('config/scenarios/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waxy',
    maintainer_email='waxy@todo.todo',
    description='文山项目 - 场景生成与AIS模拟一体化管理包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wenshan_scenario = wenshan_pro.wenshan_scenario_node:main',
        ],
    },
)
