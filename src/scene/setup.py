from glob import glob
from setuptools import find_packages, setup

package_name = "scene"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # lar ROS finne pakken
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # ta med launch-filer i install-treet
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sebastian",
    maintainer_email="sebastmy@stud.ntnu.no",
    description="Interactive cubes + table published as TF frames",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # format:  <kommando> = <pythonmodul>:<main-funksjon>
            "interactive_scene = scene.interactive_scene:main"
            # legg til flere noder her etter behov
        ],
    },
)