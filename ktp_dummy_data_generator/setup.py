import os;
from glob import glob;
from setuptools import find_packages, setup;

package_name: str = "ktp_dummy_data_generator";

setup(
    name=package_name,
    version="0.1.0",
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
    maintainer_email="201840103@email.daelim.ac.kr",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"{package_name} = {package_name}.main:main"
        ],
    },
)
