from setuptools import find_packages, setup

package_name = 'npz_loader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',     # ROS 2 Python API
        'numpy',     # for .npz-innlasting
    ],
    zip_safe=True,
    maintainer='hassan',
    maintainer_email='hassan@example.com',
    description='A ROS 2 node for loading and printing .npz files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_npz = npz_loader.load_npz_node:main',
        ],
    },
)

