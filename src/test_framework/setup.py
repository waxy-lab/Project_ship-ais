import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'test_framework'

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
    description='测试评估框架 - 批量运行测试场景并统计性能指标',
    license='Apache-2.0',
    tests_require=['pytest', 'hypothesis'],
    entry_points={
        'console_scripts': [
            # 测试运行器（后续实现）
            # 'test_runner = test_framework.test_runner:main',
        ],
    },
)
