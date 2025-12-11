from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rl_controller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/model', glob('model/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rl_joint_controller = rl_controller_pkg.rl_joint_controller:main',
            'joint_delta_controller = rl_controller_pkg.joint_delta_controller:main',
        ],
    },
)
