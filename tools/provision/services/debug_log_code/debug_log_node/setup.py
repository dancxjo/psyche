from setuptools import setup

package_name = 'debug_log_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='psyche',
    maintainer_email='devnull@example.com',
    description='Debug log -> voice bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'debug_log_node = debug_log_node.debug_log_node:main'
        ],
    },
)
from setuptools import setup

package_name = 'debug_log_node'

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
    maintainer='you',
    maintainer_email='you@example.com',
    description='Debug log to voice node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'debug_log_node = debug_log_node.debug_log_node:main'
        ],
    },
)
