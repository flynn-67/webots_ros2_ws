from setuptools import setup

package_name = 'turtlebot3_camera_service'

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
    description='Turtlebot3 camera image capture service package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run turtlebot3_camera_service image_service_server
            'image_service_server = turtlebot3_camera_service.image_service_server:main',
        ],
    },
)
