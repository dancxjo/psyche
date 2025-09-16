from setuptools import setup

package_name = 'psyched_ublox_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='psyched',
    maintainer_email='you@example.com',
    description='Launch wrapper for ublox GPS node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
