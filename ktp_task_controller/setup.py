import os;
from glob import glob;
from setuptools import find_packages, setup;

package_name: str = "ktp_task_controller";

setup(
    name=package_name,
    version="0.1.1",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reidlo",
    maintainer_email="naru5135@wavem.net",
    description="KTP Task Controller Package",
    license="None",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"{package_name} = {package_name}.main:main"
        ],
    },
)
