from setuptools import setup
import os
from glob import glob

package_name = 'voxnav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'open3d', 'PyQt5'],
    zip_safe=True,
    maintainer='nafis',
    maintainer_email='nafis@example.com',
    description='ROS2 package for crowd simulation using Recast/Detour navigation mesh',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulate_crowd = voxnav.simulate_crowd:main',
            'simulate_crowd_cosim = voxnav.simulate_crowd_cosim:main',
            'fake_robot_pose = voxnav.fake_robot_pose:main',
            'navmesh_baker = voxnav.navmesh_baker_app:main',
        ],
    },
)
