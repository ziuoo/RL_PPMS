from setuptools import find_packages, setup

package_name = 'coordinate_transformation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiwoo',
    maintainer_email='jiwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pixel_to_world=coordinate_transformation.pixel2world:main',
            'move_to_point_server = coordinate_transformation.move_to_point_server:main',
        ],
    },
)
