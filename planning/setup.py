from setuptools import find_packages, setup
import os
from glob import glob
package_name = "planning"
setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kang",
    maintainer_email="kasperg3@rocketmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": ["planner = planning.planner:main"],
    },
)
