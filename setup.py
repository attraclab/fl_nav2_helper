from setuptools import setup
import os
from glob import glob

package_name = 'fl_nav2_helper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='Just a helper function to use fast-lio Odometry in Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'relay_topics = fl_nav2_helper.relay_topics:main',
        'nav2_path_following = fl_nav2_helper.nav2_path_following:main',
        ],
    },
)
