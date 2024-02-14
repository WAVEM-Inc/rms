import paho.mqtt.client as mqtt
from rclpy.node import Node
from typing import Any

class Client:
    def __init__(self, rclpy_node: Node) -> None:
        self.__rclpy_node: Node = rclpy_node;
        
        __broker_address: str = "localhost";
        __broker_port: int = 1883;
        __client_name: str = "";
        __client_keep_alive: int = 60;
        
        self.client: mqtt.Client = mqtt.Client(client_id=__client_name, clean_session=True, userdata=None, transport="tcp");
        
        self.client.on_connect = self.__on_connect__
        self.client.on_message = self.__on_message__
        self.client.connect(host=__broker_address, port=__broker_port, keepalive=__client_keep_alive)

        if self.client.is_connected:
            self.__rclpy_node.get_logger().info("===== MQTT connected =====")
            self.client.loop_start()
        else:
            self.__rclpy_node.get_logger().error("===== MQTT failed to connect =====")
        
    
    def __on_connect__(self, client: Any, user_data: Any, flags: Any, rc: Any) -> None:
        if rc == 0:
            self.__rclpy_node.get_logger().info("===== MQTT connection succeeded result code : [{}] =====".format(str(rc)))
        else:
            self.__rclpy_node.get_logger().error("===== MQTT connection failed result code : [{}] =====".format(str(rc)))
            
    def __on_message__(self, client: Any, user_data: Any, msg: Any) -> None:
        self.__rclpy_node.get_logger().info("MQTT received message : {}".format(msg.payload.decode()))
        
    def publish(self, topic: str, payload: Any) -> None:
        self.client.publish(topic=topic, payload=payload, qos=0)


    def subscribe(self, topic: str) -> None:        
        self.__rclpy_node.get_logger().info("MQTT granted subscription from [{}]".format(topic))
        self.client.subscribe(topic=topic, qos=0)


__all__ = ["rcs_mqtt_bridge.mqtt.client"]