import os
from setuptools import find_packages, setup

package_name = 'thaiquiri_webots'


def package_files(source_dir, target_dir):
    """
    Recursively finds all files in a source directory and maps them
    to the target path in the installation share space.
    """
    paths = []
    for (path, directories, filenames) in os.walk(source_dir):
        for filename in filenames:
            # Source file full path
            source_file = os.path.join(path, filename)
            # Replicate the internal folder structure relative to the source_dir
            relative_path = os.path.relpath(path, source_dir)
            if relative_path == '.':
                dest_dir = target_dir
            else:
                dest_dir = os.path.join(target_dir, relative_path)

            paths.append((dest_dir, [source_file]))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    # ('share/' + package_name + '/launch', ['launch/drone_launch.py']),
    ('share/' + package_name + '/webots_world/worlds',
     ['webots_world/worlds/mine_thaiquiri.wbt']),
    ('share/' + package_name + '/webots_world/protos',
     ['webots_world/protos/thaiquiri.proto']),
    ('share/' + package_name + '/resource', ['resource/thaiquiri.urdf']),
    ('share/' + package_name, ['package.xml']),
]

meshes_target = os.path.join(
    'share', package_name, 'webots_world/protos/meshes')
data_files.extend(package_files('webots_world/protos/meshes', meshes_target))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'drone_driver = thaiquiri_webots.drone_driver:main',
            # 'teleop_drone = thaiquiri_webots.teleop_drone:main',
        ],
    },
)
