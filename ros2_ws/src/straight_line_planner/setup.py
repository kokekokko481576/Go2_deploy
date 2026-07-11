import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'straight_line_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koga_koichiro',
    maintainer_email='koga_koichiro@naoe.eng.osaka-u.ac.jp',
    description=(
        '経路生成計画 Phase1 M1: 自己位置から目標作業姿勢までの直線補間Pathを出すノード'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_line_planner_node = straight_line_planner.straight_line_planner_node:main',
        ],
    },
)
