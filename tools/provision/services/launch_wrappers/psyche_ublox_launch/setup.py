from setuptools import setup

package_name = 'psyche_ublox_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] if False else [],
    data_files=[
        ('share/psyche_ublox_launch/launch', ['launch/ublox_launch.py']),
        ('share/psyche_ublox_launch/params', ['params/ublox_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='you',
    description='Launch wrapper for ublox',
)
