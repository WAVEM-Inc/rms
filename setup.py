from setuptools import setup, find_packages

package_name: str = "rmde"
base_package_dir: str = "src"

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(package_name, include=('*')),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_dir={"": base_package_dir},
    install_requires=['setuptools'],    
    zip_safe=True,
    maintainer='reidlo',
    maintainer_email='naru5135@wavem.net',
    description='RMS MQTT Data Exchanger',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "exchanger = main:main"
        ],
    },
)
