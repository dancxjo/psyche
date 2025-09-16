from setuptools import setup

package_name = 'psyche_create_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/psyche_create_launch/launch', ['launch/create_launch.py']),
        ('share/psyche_create_launch/params', ['params/create_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='you',
    description='Launch wrapper for create driver',
)
