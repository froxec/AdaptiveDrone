from setuptools import find_packages, setup

package_name = 'mpc_control'

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
    maintainer='Piotr Durawa',
    maintainer_email='durawa.p.soft@gmail.com',
    description='Control package for ArduCopter',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = mpc_control.mpc_node:main'
        ],
    },
)
