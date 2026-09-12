from setuptools import setup

package_name = 'd1_arm_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riku',
    maintainer_email='riku062214riku@gmail.com',
    description='到達通知を受けてD1アームを決め打ち角度へ動かす',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_demo_node = d1_arm_demo.arm_demo_node:main',
        ],
    },
)
