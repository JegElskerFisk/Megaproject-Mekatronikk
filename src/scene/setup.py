from setuptools import setup
import os
from glob import glob

setup(
    name='scene',
    version='0.0.1',
    packages=['scene'],
    install_requires=['setuptools'],
    data_files=[
        ('share/scene', ['package.xml']),
        ('share/scene/launch', glob('launch/*.py')),
        ('share/ament_index/resource_index/packages', ['resource/scene']),
    ],
    entry_points={
        'console_scripts': [
            'interactive_scene = scene.interactive_scene:main',
            'table_scene = scene.table_scene:main',
        ],
    },
)