import os
from glob import glob
from setuptools import setup

package_name: str = 'rmde'
mqtt_package_name: str = package_name + '.mqtt'
node_package_name: str = package_name + '.node'

rms_package_name: str = package_name + '.rms'

rms_common_package_name: str = rms_package_name + '.common'

rms_request_package_name: str = rms_package_name + '.request'
rms_request_config_package_name: str = rms_request_package_name + '.config'
rms_request_control_package_name: str = rms_request_package_name + '.control'
rms_request_path_package_name: str = rms_request_package_name + '.path'

rms_response_package_name: str = rms_package_name + '.response'
rms_response_control_event_package_name: str = rms_response_package_name + '.control_event'
rms_response_location_package_name: str = rms_response_package_name + '.location'
rms_response_status_event_package_name: str = rms_response_package_name + '.status_event'
rms_response_task_event_package_name: str = rms_response_package_name + '.task_event'


packages_list: list = [
    package_name, 
    mqtt_package_name, 
    node_package_name, 
    rms_package_name, 
    rms_common_package_name,
    rms_request_package_name,
    rms_request_config_package_name,
    rms_request_control_package_name,
    rms_request_path_package_name,
    rms_response_package_name,
    rms_response_control_event_package_name,
    rms_response_location_package_name,
    rms_response_status_event_package_name,
    rms_response_task_event_package_name
]

setup(
    name=package_name,
    version='0.2.0',
    packages=packages_list,
    include_package_data=True,
    package_data={
        mqtt_package_name: ['mqtt.ini'],
        rms_common_package_name: ['config.ini']
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],    
    zip_safe=True,
    maintainer='reidlo',
    maintainer_email='naru5135@wavem.net',
    description='RMS MQTT Data Exchanger',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmde_executor = rmde.main:main'
        ],
    },
)
