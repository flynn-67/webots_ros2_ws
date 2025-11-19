from setuptools import setup

package_name = 'nav2_bt_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flynn',
    maintainer_email='youdongoh67@gmail.com',
    description='Nav2 homework BT sequence node using /bt/goal_pose.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_sequence_node = nav2_bt_hw.bt_sequence_node:main',
        ],
    },
)
