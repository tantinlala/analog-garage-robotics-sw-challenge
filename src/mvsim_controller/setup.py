from setuptools import find_packages, setup

package_name = 'mvsim_controller'

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
    maintainer='rosdev',
    maintainer_email='nicholas.tantisujjatham@gmail.com',
    description='Simple script to command mvsim robot at present speed limit',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_mvsim_controller = mvsim_controller.node_mvsim_controller:main'
        ],
    },
)
