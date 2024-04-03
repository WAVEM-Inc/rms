import os;
import sys;
from setuptools import setup, find_packages;
from glob import glob;

package_name: str = "ktp_interface";
tcp_libs_dir: str = f"{package_name}.tcp.libs";
tcp_lib_name: str = "";

if sys.version_info >= (3, 0):
    tcp_lib_name = "IotmakersStdDevicePy3.so";
else:
    tcp_lib_name = "IotmakersStdDevicePy2.so";

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + ".*"]),
    package_data={
        tcp_libs_dir: [tcp_lib_name]
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reidlo',
    maintainer_email='naru5135@wavem.net',
    description='KEC RMS Interface Package',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"{package_name} = {package_name}.main:main"
        ],
    },
);
