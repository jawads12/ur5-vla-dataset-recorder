from glob import glob
from setuptools import find_packages, setup


package_name = "ur5_vla_dataset_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Grasp Lab",
    maintainer_email="maintainer@example.com",
    description="Safe synchronized staging-dataset recorder for UR5 VLA demonstrations.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dataset_recorder = ur5_vla_dataset_recorder.recorder_node:main",
        ],
    },
)

