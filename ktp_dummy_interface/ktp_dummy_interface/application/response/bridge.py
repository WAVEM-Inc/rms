from rclpy.node import Node;

from ktp_dummy_interface.application.mqtt import Client;


class ResponseBridge:

    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__mqtt_client: Client = mqtt_client;


__all__ = ["ResponseBridge"];