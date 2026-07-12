from setuptools import find_packages, setup

package_name = 'cmd_vel_safety'

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
        '経路追従計画M1: 速度・加速度クランプとcmd_velウォッチドッグ(0.5s)'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_safety_node = cmd_vel_safety.cmd_vel_safety_node:main',
        ],
    },
)
