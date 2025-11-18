from setuptools import find_packages, setup

package_name = 'ais_to_mqtt'

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
    maintainer='waxy',
    maintainer_email='waxy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mqtt_bridge = ais_to_mqtt.mqtt_bridge_node:main',
            'ais_file_test = ais_to_mqtt.ais_file_test_node:main',
            'ais_reader = ais_to_mqtt.ais_reader_node:main',
            'analyzeData = ais_to_mqtt.analyzeData:main'
        ],
    },
)
