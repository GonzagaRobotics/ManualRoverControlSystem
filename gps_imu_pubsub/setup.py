from setuptools import setup

package_name = 'gps_imu_pubsub'

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
    maintainer='root',
    maintainer_email='eleehiga@proton.me',
    description='pub sub gps imu',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_imu_pub = gps_imu_pubsub.publisher_member_function:main',
        ],
    },
)
