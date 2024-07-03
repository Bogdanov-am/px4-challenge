import os
from glob import glob
from setuptools import setup

package_name = 'px4_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=[
        'setuptools',
        'filterpy==1.4.5'
        ],
    zip_safe=True,
    maintainer='Anton Bogdanov',
    maintainer_email='bogdan0v.am@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = px4_challenge.pose_publisher:main',
            'target_follower = px4_challenge.target_follower:main'
        ],
    },
)
