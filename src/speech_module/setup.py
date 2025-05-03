from setuptools import find_packages, setup

package_name = 'speech_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/resource', ['resource/en_US-amy-low.onnx', 'resource/en_US-amy-low.onnx.json']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkdev',
    maintainer_email='Darkness_Linux',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_node = speech_module.main:main',
        ],
    },
)
