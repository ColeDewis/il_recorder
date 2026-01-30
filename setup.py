import os
from glob import glob

from setuptools import find_packages, setup

package_name = "il_recorder"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "configs/robots"),
            glob("configs/robots/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "configs/observations"),
            glob("configs/observations/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="coledewis@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "il_recorder = il_recorder.il_recorder:main",
            "inspect_hdf5 = il_recorder.scripts.inspect_hdf5:main",
            "pc_check = il_recorder.scripts.pc_check:main",
            "keyboard_joy = il_recorder.scripts.keyboard_joy:main",
        ],
    },
)
