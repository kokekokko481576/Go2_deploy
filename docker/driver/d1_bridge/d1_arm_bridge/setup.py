from setuptools import setup

package_name = 'd1_arm_bridge'

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
    description='arm_command を D1-T の rt/arm_Command へ変換して送る',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_bridge_node = d1_arm_bridge.arm_bridge_node:main',
        ],
    },
)
