import os;
from setuptools import setup, find_packages;
from glob import glob;

package_name: str = "ktp_data_manager";

setup(
    name=package_name,
    version="0.1.3",
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
    description="KTP Data Manager Package",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"{package_name} = {package_name}.main:main"
        ],
    },
);
