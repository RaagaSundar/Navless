from glob import glob
from setuptools import setup

package_name = 'navless_flight_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Navless',
    maintainer_email='noreply@example.com',
    description='First-flight tests (hover, yaw, forward) through skybeta_flight_interface: setpoints only, the pilot arms.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['basic_flight = navless_flight_tests.basic_flight:main']},
)
