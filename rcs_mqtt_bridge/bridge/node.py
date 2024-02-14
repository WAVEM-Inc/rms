import json
import paho.mqtt.client as mqtt

from rclpy.node import Node

from typing import Any
from typing import Dict

from ..mqtt.client import Client

NODE_NAME: str = "rcs_mqtt_bridge"

class RcsMQTTBridge(Node):
    
    def __init__(self) -> None:
        super().__init__(NODE_NAME);
        
        self.get_logger().info(f"{NODE_NAME} created");
        
        self.__mqtt_client: Client = Client(self);
        
        self.__ros_message_init_topic: str = "ros_message_init";
        self.__nuc_shutdown_topic: str = "nuc_shutdown";
        
        self.__ros_message_init_sub();
    
    def __ros_message_init_sub(self) -> None:       
        def __ros_message_init_sub_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()
                mqtt_json: Any = json.loads(mqtt_message.payload)
                
                self.get_logger().info(f'[{mqtt_topic}] subscription cb json [{mqtt_decoded_payload}]');
                
            except KeyError as ke:
                self.get_logger().error(f'Invalid JSON Key in MQTT [{mqtt_topic}] subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.get_logger().error(f'Invalid JSON format in MQTT [{mqtt_topic}] subscription callback: {jde.msg}')

            except Exception as e:
                self.get_logger().error(f'Exception in MQTT [{mqtt_topic}] subscription callback: {e}')
                raise
            
        self.__mqtt_client.subscribe(topic=self.__ros_message_init_topic);
        self.__mqtt_client.client.message_callback_add(sub=self.__ros_message_init_topic, callback=__ros_message_init_sub_cb);