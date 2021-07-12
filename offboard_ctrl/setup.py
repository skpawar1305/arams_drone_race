import os
from glob import glob
from setuptools import setup

package_name = 'offboard_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skpawar1305',
    maintainer_email='skpawar1305@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_ctrl = offboard_ctrl.manual_ctrl:main',
            'automatic_ctrl = offboard_ctrl.automatic_ctrl:main',
        ],
    },
)
