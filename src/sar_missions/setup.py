from setuptools import setup
import os
from glob import glob

package_name = 'sar_missions'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qpisco',
    maintainer_email='qpisco@todo.todo',
    description='SAR drone mission scripts and validation tools',
    license='Apache-2.0',
    extras_require={
    'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        'topic_rate_monitor = sar_missions.health_monitor:main',
        ],
    },
)
