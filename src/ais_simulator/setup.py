import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ais_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.geojson')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waxy',
    maintainer_email='user@todo.todo',
    description='AIS Data Simulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这行定义了我们的可执行文件
            'ais_sim_node = ais_simulator.ais_sim_node:main',
        ],
    },
)