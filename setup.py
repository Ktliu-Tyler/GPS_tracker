from setuptools import find_packages, setup
import glob

package_name = 'gps_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('gps_tracker/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docker',
    maintainer_email='liutyler124@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'gps_logger = gps_tracker.gps_logger_node:main',
        'gps_visualizer = gps_tracker.gps_visualizer_node:main',
        'gps_test_pub = gps_tracker.gps_test_publisher:main',
        'gps_animate = gps_tracker.gps_animate:main',
        'gps_animate_imd = gps_tracker.gps_animate_imd:main',
        'gps_google_map = gps_tracker.gps_google_map:main',
        'gps_csv_logger = gps_tracker.gps_csv_logger:main',
        'gps_can_pub = gps_tracker.gps_can_pub:main',
        'nmea_ntrip_driver = gps_tracker.nmea_ntrip_driver:main',
        'timer_can_pub = gps_tracker.timer_can_pub:main'
    ],
},
)
