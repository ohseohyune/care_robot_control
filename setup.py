from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'care_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seohy',
    maintainer_email='ohseohyun0531@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'neck_controller = care_robot_control.neck_controller_node:main',
            'ebimu = care_robot_control.ebimu_node:main',
            'neck_swing_demo = care_robot_control.neck_swing_demo:main',
        ],
    },
)
