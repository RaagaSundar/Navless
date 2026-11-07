from glob import glob
from setuptools import setup

package_name = 'navless_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Raaga Sundar',
    maintainer_email='rsundar31@gatech.edu',
    description='Navless DroneHacks competitor node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'course_mission = navless_nav.course_mission:main',
            'mirror_relay = navless_nav.mirror_relay:main',
        ],
    },
)
