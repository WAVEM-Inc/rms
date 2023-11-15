import os
import json
import paho.mqtt.client as mqtt

from configparser import ConfigParser
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from gts_navigation_msgs.msg import NavigationControl

from ....mqtt import mqtt_client
from ...common.service import ConfigService

from .domain import Control
from .domain import ControlCmd

from typing import Any
from typing import Dict


class ControlRequestHandler():
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: mqtt_client.Client) -> None:
        self.script_directory: str = os.path.dirname(os.path.abspath(__file__))
        self.config_file_path: str = '../../../mqtt/mqtt.ini'
        self.config_service: ConfigService = ConfigService(self.script_directory, self.config_file_path)
        self.config_parser: ConfigParser = self.config_service.read()
        self.mqtt_control_subscription_topic: str = self.config_parser.get('topics', 'control')
        
        self.rclpy_node: Node = rclpy_node
        
        self.rclpy_gts_navigation_control_publisher_name: str = '/gts_navigation/control'
        self.rclpy_gts_navigation_control_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup()
        self.rclpy_gts_navigation_control_publisher: Publisher = self.rclpy_node.create_publisher(
            msg_type = NavigationControl,
            topic = self.rclpy_gts_navigation_control_publisher_name,
            qos_profile = qos_profile_system_default,
            callback_group = self.rclpy_gts_navigation_control_publisher_cb_group
        )
        
        self.mqtt_broker: mqtt_client.Client = mqtt_broker
        self.control: Control = Control()
        self.control_cmd: ControlCmd = ControlCmd()
        
    
    def request_to_uvc(self) -> None:
        def mqtt_control_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
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
            
            self.__judge_control_cmd__()
            
        self.mqtt_broker.subscribe(topic = self.mqtt_control_subscription_topic, qos = 1)
        self.mqtt_broker.client.message_callback_add(self.mqtt_control_subscription_topic, mqtt_control_subscription_cb)
        

    def __judge_control_cmd__(self) -> None:
        is_cmd_ready: bool = (self.control_cmd.ready == True)
        is_cmd_move: bool = (self.control_cmd.move == True)
        is_cmd_stop: bool = (self.control_cmd.stop == True)
            
        if (is_cmd_ready and is_cmd_move and is_cmd_stop):
            return
        elif (is_cmd_ready and is_cmd_move):
            return
        elif (is_cmd_move and is_cmd_stop):
            return
        elif (is_cmd_ready and is_cmd_stop):
            return
        elif (is_cmd_stop and not is_cmd_ready and not is_cmd_move):
            self.rclpy_node.get_logger().info("judge gts_navigation/control's command is stop")
            
            gts_navigation_control: NavigationControl = NavigationControl()
            gts_navigation_control.cancel_navigation = True
            gts_navigation_control.resume_navigation = False
            
            self.rclpy_gts_navigation_control_publisher.publish(gts_navigation_control)
        elif (is_cmd_move and not is_cmd_stop and not is_cmd_ready):
            self.rclpy_node.get_logger().info("judge gts_navigation/control's command is move")
            
            gts_navigation_control: NavigationControl = NavigationControl()
            gts_navigation_control.cancel_navigation = False
            gts_navigation_control.resume_navigation = True
            
            self.rclpy_gts_navigation_control_publisher.publish(gts_navigation_control)
        else: return
            
            

__all__ = ['control_request_handler']
