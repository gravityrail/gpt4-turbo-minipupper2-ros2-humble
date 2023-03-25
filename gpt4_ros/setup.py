from setuptools import setup

package_name = 'gpt4_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Herman Ye',
    maintainer_email='hermanye233@icloud.com',
    description='GPT-4 Interfaces for ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpt_ros2_server = gpt4_ros.gpt_ros2_server:main',
            'gpt_ros2_client = gpt4_ros.gpt_ros2_client:main',
        ],
    },
)
