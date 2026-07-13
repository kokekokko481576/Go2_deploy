import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'go2_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'map'), glob('config/map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koga_koichiro',
    maintainer_email='koga_koichiro@naoe.eng.osaka-u.ac.jp',
    description=(
        '自己位置推定練習用(自M1・自M2): EKF・pointcloud_to_laserscan・AMCLのbringup'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
