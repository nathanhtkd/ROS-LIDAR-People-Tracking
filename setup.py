from setuptools import find_packages, setup
import glob

package_name = 'lidar_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Hartojo',
    maintainer_email='nathanhartojo@gmail.com',
    description='TODO: lidar bag processing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processor = lidar_listener.lidar_processor:main',
            'point_cluster = lidar_listener.point_cluster:main',
            'person_tracker = lidar_listener.person_tracker:main'
        ],
    },
)
