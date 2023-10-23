import rclpy

from rclpy.node import Node
from mqtt import broker
from rms.response.event.handler.event_response_handler import EventResponseHandler
import paho.mqtt.client as mqtt

from typing import List
from typing import Dict

class rmde_node(Node):
    node_name: str = 'rmde'
    rclpy_flag: str = 'RCLPY'
    mqtt_flag: str = 'MQTT'
    mqtt_broker: broker.mqtt_broker = broker.mqtt_broker()
    event_response_handler: EventResponseHandler = EventResponseHandler(mqtt_broker)
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.get_logger().info('===== {} [{}] created ====='.format(self.rclpy_flag, self.node_name))
        rclpy_timer_loop: float = 1.0
        self.create_timer(rclpy_timer_loop, self.__from_uvc_to_rms__)
        self.__from_rms_to_uvc__()
    
    
    def __from_uvc_to_rms__(self) -> None:
        self.event_response_handler.response_to_uvc()
        
    
    def __from_rms_to_uvc__(self) -> None:
        mqtt_topic_list: List = ['hubilon/atcplus/rms/path', 'hubilon/atcplus/rms/control', 'hubilon/atcplus/rms/config']
        
        def mqtt_subscription_callback(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()

            self.get_logger().info("{} MQTT received message [{}] from [{}]".format(self.mqtt_flag, mqtt_decoded_payload, mqtt_topic))
        
        for mqtt_topic in mqtt_topic_list:
            self.get_logger().info('{} [{}] subscription granted'.format(self.mqtt_flag, mqtt_topic))
            self.mqtt_broker.client.subscribe(mqtt_topic)
            self.mqtt_broker.client.message_callback_add(mqtt_topic, mqtt_subscription_callback)
        


__all__ = ["rmde_node"]