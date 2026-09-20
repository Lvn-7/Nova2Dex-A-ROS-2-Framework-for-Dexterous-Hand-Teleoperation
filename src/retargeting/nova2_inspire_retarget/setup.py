import os
from glob import glob

from setuptools import find_packages, setup


package_name = "nova2_inspire_retarget"
model_files = []
for root, _directories, files in os.walk("models"):
    if files:
        model_files.append(
            (os.path.join("share", package_name, root), [os.path.join(root, name) for name in files])
        )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ] + model_files,
    install_requires=["setuptools", "mujoco>=3.0"],
    zip_safe=False,
    maintainer="jiimmy",
    maintainer_email="1131359622@qq.com",
    description="Independent Nova2 to Inspire direct retargeting and MuJoCo simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nova2_inspire_retarget_node = nova2_inspire_retarget.retarget_node:main",
            "inspire_mujoco_viewer = nova2_inspire_retarget.mujoco_viewer:main",
        ],
    },
)
