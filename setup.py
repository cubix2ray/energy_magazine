from setuptools import setup

package_name = 'energy_magazine'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'current_load = energy_magazine.current_load:main',
        ],
    },
)