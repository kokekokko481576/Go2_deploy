from setuptools import find_packages, setup

package_name = 'fake_localization_sensors'

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
    maintainer='kokko',
    maintainer_email='kogakou.k@gmail.com',
    description=(
        '自己位置推定練習用: ダミーのOdometry/IMUをpublishする最小ノード'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_odom_imu_node = fake_localization_sensors.fake_odom_imu_node:main'
        ],
    },
)
