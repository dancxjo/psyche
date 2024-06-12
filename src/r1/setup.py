from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'r1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dancxjo',
    maintainer_email='tdreed@gmail.com',
    description='Robot1 (r1) is a reference robot host for psyche',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "play_song = r1.play_song:main",
            "motivate = r1.motivator:main",
            "monitor = r1.monitor:main",
            "exec_shell = r1.shell_exec:main",
            "announce_boot = r1.boot_announcer:main",
        ],
    },
)
