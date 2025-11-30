import os
from glob import glob
from setuptools import setup

package_name = 'car_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Instala el package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- 1. Instala los archivos de lanzamiento ---
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),

        # --- 2. Instala los mundos de Webots ---
        (os.path.join('share', package_name, 'world'),
            glob(os.path.join('world', '*.wbt'))),
            
        # --- 3. Instala los archivos de recursos ---
        (os.path.join('share', package_name, 'resource'),
            glob(os.path.join('resource', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='haoxiang.liu@alumnos.upm.es',
    description='Paquete ROS 2 para control de coche autónomo en Webots.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    
    # --- Define los ejecutables ---
    entry_points={
        'console_scripts': [
            'car_controller = car_pkg.car_controller:main',
            'lane_controller = car_pkg.lane_controller:main',
            'sign_detector = car_pkg.sign_detector:main',
        ],
    },
)
