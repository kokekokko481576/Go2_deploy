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
        # 実機操作の補助スクリプト。`ros2 run go2_sport_bridge estop.sh` で呼べるよう
        # lib/<パッケージ名> に置く(ros2 run が実行ファイルを探す場所)。
        ('lib/' + package_name, ['scripts/estop.sh', 'scripts/jog.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koga_koichiro',
    maintainer_email='koga_koichiro@naoe.eng.osaka-u.ac.jp',
    description=(
        'cmd_velをGo2 Sport Mode APIのMove命令へ変換する中継ノード'
        '(逆方向: sportmodestateをOdometry/Imuへ変換する中継ノードも含む)'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_sport_node = go2_sport_bridge.cmd_vel_to_sport_node:main',
            'state_to_odom_imu_node = go2_sport_bridge.state_to_odom_imu_node:main',
            'utlidar_cloud_restamp_node = go2_sport_bridge.utlidar_cloud_restamp_node:main',
        ],
    },
)
