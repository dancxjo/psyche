from setuptools import setup

package_name = 'psyche_mic_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/psyche_mic_launch/launch', ['launch/mic_launch.py']),
        ('share/psyche_mic_launch/params', ['params/mic_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='you',
    description='Launch wrapper for mic_node',
)
