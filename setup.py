from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'iic2685_proyecto_final_grupo_7_2025'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcifuentesf',
    maintainer_email='mcifuentesf@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = iic2685_proyecto_final_grupo_7_2025.robot_controller:main',
            'map_manager = iic2685_proyecto_final_grupo_7_2025.map_manager:main',
            'localization_helper = iic2685_proyecto_final_grupo_7_2025.localization_helper:main',
            'navigation_helper = iic2685_proyecto_final_grupo_7_2025.navigation_helper:main',
        ],
    },
)
