from setuptools import setup

package_name = 'energy_magazine_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/matlab/', [package_name + '/battery_data/B0007.mat'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='',
    author_email='',
    maintainer='',
    maintainer_email='',
    keywords=['ROS'],
    description='Nodes made for university classes',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'current_load = ' + package_name + '.current_load:main',
            'random_number = ' + package_name + '.random_number_gen:main'
            'battery_state_server = ' + package_name + '.battery_state_server:main'
        ],
    },
)