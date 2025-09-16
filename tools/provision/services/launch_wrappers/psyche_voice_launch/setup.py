from setuptools import setup

package_name = 'psyche_voice_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/psyche_voice_launch/launch', ['launch/voice_launch.py']),
        ('share/psyche_voice_launch/params', ['params/voice_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='you',
    description='Launch wrapper for voice_node',
)
