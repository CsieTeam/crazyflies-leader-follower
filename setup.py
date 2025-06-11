from setuptools import setup

import os
from glob import glob

package_name = 'crazyflie_leader_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Crazyflie leader-follower example',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = crazyflie_leader_follower.run:main',
            'leader_node = crazyflie_leader_follower.leader_node:main',
            'follower_node = crazyflie_leader_follower.follower_node:main',
        ],
    },
)

