from setuptools import find_packages, setup
import os

package_name = 'wireless_scanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AntoArroyo',
    maintainer_email='arroyo.antom@gmail.com',
    description='A package for scanning Wi-Fi and Bluetooth devices',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wireless_scanner = wireless_scanner.wireless_scanner:main',
        ],
    },
)
