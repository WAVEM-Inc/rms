import json
import paho.mqtt.client as mqtt

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbridge_library.internal import message_conversion

from ....mqtt import mqtt_client

from .domain import Control
from .domain import ControlCmd

from typing import Any
from typing import Dict


class ControlRequestHandler():
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    
    mqtt_path_subscription_topic: str = 'hubilon/atcplus/rms/rco0000000/rbt00000000/control'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.rclpy_node: Node = rclpy_node
        self.mqtt_broker: mqtt_client.Client = mqtt_broker
        self.control: Control = Control()
        self.control_cmd: ControlCmd = ControlCmd()
        
    
    def request_to_uvc(self) -> None:
        def mqtt_path_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()
            mqtt_json: Any = json.loads(mqtt_message.payload)
            
            self.rclpy_node.get_logger().info('{} subscription cb payload [{}] from [{}]'.format(self.mqtt_flag, mqtt_decoded_payload, mqtt_topic))
            self.rclpy_node.get_logger().info('{} subscription cb json [{}] from [{}]'.format(self.mqtt_flag, mqtt_json, mqtt_topic))

            self.control = Control(
                header = mqtt_json['header'],
                controlCmd = mqtt_json['controlCmd']
            )
            
            self.control_cmd = ControlCmd(
                ready = self.control.controlCmd['ready'],
                move = self.control.controlCmd['move'],
                stop = self.control.controlCmd['stop']
            )
            
        self.mqtt_broker.subscribe(topic = self.mqtt_path_subscription_topic, qos = 1)
        self.mqtt_broker.client.message_callback_add(self.mqtt_path_subscription_topic, mqtt_path_subscription_cb)