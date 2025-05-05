from setuptools import find_packages, setup

package_name = 'mqtt_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['mqtt_module'],exclude=['-0.0.0-py3.1']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='mak',
    maintainer_email='abdullah_boobak@hotmail.com',
    description='MQTT communication package for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_node = mqtt_module.main:main',
        ],
    },
)