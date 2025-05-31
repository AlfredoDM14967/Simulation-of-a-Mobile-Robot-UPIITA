from setuptools import find_packages, setup

package_name = 'nodosros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nodos.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'IHM_remota = nodosros2.IHM_remota:main',
            'rover_IHMremota = nodosros2.rover_IHMremota:main',
            'Camara = nodosros2.Camara:main',
            'Controller_manager = nodosros2.Controller_manager:main',
            'GPS = nodosros2.GPS:main',
            'LIDAR = nodosros2.LIDAR:main',
            'motores = nodosros2.Motores:main',
            'scan_sub = nodos_ros2.scan_sub:main',
            'STM = nodosros2.STM:main',
        ],
    },
)
