from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'psyche'

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
    description='Psyche is an operating system for robots',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lpu = psyche.language_processor:main',
            'vlpu = psyche.vision_lp:main',
            'distill = psyche.distiller:main',
            'listen_for_speech = psyche.listen:main',
            'sense = psyche.sense:main',
            'heartbeat = psyche.heartbeat:main',
            'transcribe_speech = psyche.transcribe:main',
            'informant = psyche.informant:main',
            'memorialist = psyche.memorialist:main',
            'identity = psyche.identity:main',
            'stream_voice = psyche.stream:main',
            'stream_text = psyche.voice_stream_text:main',
            'watch = psyche.continuous_vision:main',
            'integrate = psyche.integration:main',
        ],
    },
)
