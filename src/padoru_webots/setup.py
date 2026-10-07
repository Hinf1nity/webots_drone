from setuptools import find_packages, setup

package_name = 'padoru_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages',
                   ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',
                  ['launch/drone_launch.py']))
data_files.append(('share/' + package_name + '/webots_world/worlds',
                  ['webots_world/worlds/test.wbt']))
data_files.append(('share/' + package_name + '/webots_world/protos',
                   ['webots_world/protos/Mavic2Pro.proto']))
data_files.append(('share/' + package_name + '/webots_world/protos',
                   ['webots_world/protos/MyVelodynePuck.proto']))
data_files.append(('share/' + package_name + '/resource',
                  ['resource/drone_padoru.urdf']))
data_files.append(('share/' + package_name + '/resource',
                   ['resource/drone_padoru2.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

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
            'drone_driver = padoru_webots.drone_driver:main',
            'teleop_drone = padoru_webots.teleop_drone:main',
        ],
    },
)
