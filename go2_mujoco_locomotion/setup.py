from glob import glob
import os

from setuptools import find_packages, setup

package_name = "go2_mujoco_locomotion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "models"),
         [p for p in glob("models/*") if os.path.isfile(p)]),
        (os.path.join("share", package_name, "models", "go2"), glob("models/go2/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 terrain team",
    maintainer_email="maintainer@example.com",
    description="Unitree Go2 MuJoCo locomotion and terrain navigation demo",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_node = go2_mujoco_locomotion.policy_node:main",
        ],
    },
)
