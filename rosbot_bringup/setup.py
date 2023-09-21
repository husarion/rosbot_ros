import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rosbot_bringup"

setup(
    name=package_name,
    version="0.8.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Husarion",
    maintainer_email="contact@husarion.com",
    description="ROSbot 2, 2R, PRO bringup package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
