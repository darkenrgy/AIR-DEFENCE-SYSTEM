from setuptools import find_packages, setup

package_name = 'mayuri'

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
    maintainer='shreepraveen',
    maintainer_email='shreepraveen@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'classification= mayuri.ai_core.classification:main',
             #'alert_system= mayuri.ui_control.alert_system:main',
            'dashboard_ui= mayuri.ui_control.dashboard_ui:main',
            'map_visualizer= mayuri.ui_control.map_visualizer:main',
            #'ai_core',
            #('share/mayuri/gazebo/worlds', ['mayuri/gazebo/worlds/empty_world.sdf']),
            # ('share/mayuri/urdf', ['mayuri/urdf/mayuri_drone.urdf.xacro']),
        ],
    },
)
