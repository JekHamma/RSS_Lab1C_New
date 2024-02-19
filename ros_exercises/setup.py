from setuptools import setup

package_name = 'ros_exercises'

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
    maintainer='racecar',
    maintainer_email='JackHamm@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = ros_exercises.simple_publisher:main',
            'simple_subscriber = ros_exercises.simple_subscriber:main',
            'fake_scan_publisher = ros_exercises.fake_scan_publisher:main',
            'open_space_publisher = ros_exercises.open_space_publisher:main',
            'dynamic_tf_cam_publisher = ros_exercises.dynamic_tf_cam_publisher:main',
            'static_tf_cam_publisher = ros_exercises.static_tf_cam_publisher:main',
        ],
    },
)