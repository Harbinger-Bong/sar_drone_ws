from setuptools import setup
import os
from glob import glob

package_name = 'sar_nav_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qpisco',
    maintainer_email='qpisco@todo.todo',
    description='SAR drone Nav2 to MAVROS bridge node',
    license='Apache-2.0',
    extras_require={
    'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'nav2_path_to_setpoint = sar_nav_bridge.nav2_path_to_setpoint:main',
        ],
    },
)
