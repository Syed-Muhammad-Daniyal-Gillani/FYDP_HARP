from setuptools import setup
import os
from glob import glob

package_name = 'face_animations'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Face animations module for displaying emotions',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_animations = face_animations.main:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Install marker
        ('share/' + package_name, ['package.xml']),  # Install package.xml
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Install launch files
        (os.path.join('share', package_name, 'display_emotions'), glob('display_emotions/*.png')),  # Install images
    ],
    include_package_data=True,  # Ensure extra files are included
)
