from setuptools import setup
import os
from glob import glob

package_name = 'multi_drone_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', [
            'config/terrain_mapping_params.yaml',
            'config/drone_viz.rviz'
        ]),
        ('share/' + package_name + '/models/terrain', [
            'models/terrain/model.config',
            'models/terrain/model.sdf'
        ]),
        ('share/' + package_name + '/models/terrain/skyscrapper', [
            'models/terrain/skyscrapper/model.config',
            'models/terrain/skyscrapper/model.sdf'
        ]),
        ('share/' + package_name + '/models/terrain/skyscrapper/meshes', 
            glob('models/terrain/skyscrapper/meshes/*.*')),
        ('share/' + package_name + '/models/terrain/skyscrapper/meshes/interior', 
            glob('models/terrain/skyscrapper/meshes/interior/*.*')),
        ('share/' + package_name + '/models/terrain/meshes', [
            'models/terrain/meshes/artburysol175.obj',
            'models/terrain/meshes/artburysol175.mtl',
            'models/terrain/meshes/artburysol175.stl',
            'models/terrain/meshes/artburysol175_aruco.obj',
            'models/terrain/meshes/artburysol175_aruco.mtl',
        ]),
        ('share/' + package_name + '/models/terrain/perseverance_mars_pano/source', [
            'models/terrain/perseverance_mars_pano/source/marskybox.obj'
        ]),
        ('share/' + package_name + '/models/terrain/gale_crater_mars/source', [
            'models/terrain/gale_crater_mars/source/GaleCrater_2M_8K.obj',
            'models/terrain/gale_crater_mars/source/GaleCrater_2M_8K.stl',
            'models/terrain/gale_crater_mars/source/GaleCrater_2M_8K.mtl'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS2 package for executing spiral trajectory with PX4',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiral_trajectory = multi_drone_slam.spiral_trajectory:main',
            'feature_tracker = multi_drone_slam.feature_tracker:main',
            'pose_visualizer = multi_drone_slam.pose_visualizer:main',
            'my_code = multi_drone_slam.my_code:main',
            'pitch_altitude = multi_drone_slam.pitch_altitude:main',
            'dron1 = multi_drone_slam.dron1:main',
            'dron2 = multi_drone_slam.dron2:main',
            'dron3 = multi_drone_slam.dron3:main'
        ],
    },
    python_requires='>=3.8'
)