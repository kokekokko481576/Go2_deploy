from setuptools import find_packages, setup

package_name = 'go2_sport_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koga_koichiro',
    maintainer_email='koga_koichiro@naoe.eng.osaka-u.ac.jp',
    description=(
        'cmd_velをGo2 Sport Mode APIのMove命令へ変換する中継ノード'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_sport_node = go2_sport_bridge.cmd_vel_to_sport_node:main',
        ],
    },
)
