from setuptools import find_packages, setup

package_name = 'vision_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # Keeps package metadata
        ('share/' + package_name, ['package.xml']),  # Ensures package.xml is installed
        ('share/' + package_name + '/resource', ['resource/haarcascade_frontalface_default.xml']),  # INSTALLS haarcascade!
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkdev',
    maintainer_email='darkdev@todo.todo',
    description='Face tracking module for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'harp_vision = vision_module.main:main',
        ],
    },
)
