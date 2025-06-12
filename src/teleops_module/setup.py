from setuptools import find_packages, setup

package_name = 'teleops_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['teleops_module']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mak',
    maintainer_email='abdullah_boobak@hotmail.com',
    description='Teleops module for controlling the robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleops_node = teleops_module.main:main',
        ],
    },
)