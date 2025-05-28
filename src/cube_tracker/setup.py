from setuptools import find_packages, setup

package_name = 'cube_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/static_camera.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'numpy<2',            # Pin numpy to <2 to avoid your import issues
        'opencv-python',      # OpenCV Python binding
    ],
    python_requires='>=3.7',
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Cube detection and tracking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detector = cube_tracker.cube_detector:main',
            'cube_listener = cube_tracker.cube_listener:main',
            'static_cam_tf = cube_tracker.static_tf_pub:main',
            'cube_tf_republisher = cube_tracker.cube_tf_republisher:main',
        ],
    },
)

