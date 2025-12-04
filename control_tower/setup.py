from setuptools import find_packages, setup

package_name = 'control_tower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/control_tower', ['control_tower/whisper_transcribe.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frezest',
    maintainer_email='frezest@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_tower_gui = control_tower.control_tower_gui:main',
            'keyword_service_server = control_tower.keyword_service_server:main',
        ],
    },
)
