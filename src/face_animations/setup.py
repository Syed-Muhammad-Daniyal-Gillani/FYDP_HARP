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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_animations = face_animations.main:main',
        ],
    },
    data_files=[
        ('share/' + package_name + '/emotions', glob('face_animations/emotions/*.png'))
    ],
)
