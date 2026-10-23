from setuptools import setup
import os
from glob import glob

package_name = 'marker_approach'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riku',
    maintainer_email='riku062214riku@gmail.com',
    description='マーカーへの最終アプローチ制御',
    license='MIT',
    entry_points={
        'console_scripts': [
            'approach_node = marker_approach.approach_node:main',
        ],
    },
)
