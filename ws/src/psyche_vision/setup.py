from setuptools import setup

package_name = 'psyche_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/vision_launch.py', 'launch/face_object.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Psyche Robot',
    maintainer_email='psyche@robot.local',
    description='Object detection and tracking for psyche robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_detector = psyche_vision.object_detector:main',
            'object_controller = psyche_vision.object_controller:main',
        ],
    },
)