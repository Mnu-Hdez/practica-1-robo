from setuptools import setup
import os
from glob import glob

package_name = 'car_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manu',
    maintainer_email='manuel.mhernandez@alumnos.upm.es',
    description='Practica 1 Robotica',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_controller = car_package.lane_controller:main',
            'sign_detector = car_package.sign_detector:main',
            'car_driver = car_package.car_driver:main',
        ],
    },
)