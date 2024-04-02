import socket;
import paho.mqtt.client as mqtt;

from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from typing import Any;


class Client:

    def __init__(self, node: Node) -> None:
        self.client: mqtt.Client
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();

        self.__host: str = self.__node.get_parameter(name="host").get_parameter_value().string_value;
        self.__port: int = self.__node.get_parameter(name="port").get_parameter_value().integer_value;
        self.__client_name: str = self.__node.get_parameter(name="client_id").get_parameter_value().string_value;
        self.__client_keep_alive: int = self.__node.get_parameter(name="client_keep_alive").get_parameter_value().integer_value;
        self.__user_name: str = self.__node.get_parameter(name="user_name").get_parameter_value().string_value;
        self.__password: str = self.__node.get_parameter(name="password").get_parameter_value().string_value;
        self.__type: str = self.__node.get_parameter(name="type").get_parameter_value().string_value;
        self.__path: str = self.__node.get_parameter(name="path").get_parameter_value().string_value;
        self.is_connected: bool = False;

    def check_broker_opened(self) -> bool:
        try:
            sock: socket = socket.create_connection(address=(self.__host, self.__port), timeout=None);
            sock.close();
            return True;
        except socket.error as se:
            self.__log.error(f"MQTT socket error : {se}");
            pass;

        return False;

    def initialize(self) -> None:
        self.connect();
        self.run();

    def connect(self) -> None:
        try:
            self.__log.info(
                f"MQTT Connect\n"
                f"host : {self.__host}\n"
                f"port : {self.__port}\n"
                f"type : {self.__type}\n");

            self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=self.__client_name, clean_session=True, userdata=None, transport=self.__type);
                
            self.client.username_pw_set(self.__user_name, self.__password);

            self.client.on_connect = self.on_connect;
            self.client.on_disconnect = self.on_disconnect;
            self.client.on_message = self.on_message;
            self.client.connect(host=self.__host, port=self.__port, keepalive=self.__client_keep_alive);
        except OSError as ose:
            self.__log.error(f"MQTT OSError : {ose}");
        except Exception as e:
            self.__log.error(f"MQTT Error : {e}");

    def run(self) -> None:
        if self.is_connected:
            self.__log.info("MQTT Client is running");
            self.client.loop_start();
        else:
            self.__log.error("MQTT Client is not connected to broker");
            return

    def rerun(self) -> None:
        self.client.disconnect();
        self.client.loop_stop();
        self.run();

    def on_connect(self, client: Any, user_data: Any, flags: Any, rc: Any) -> None:
        if rc == 0:
            self.__log.info(f"MQTT connection succeeded result code : [{str(rc)}]");
        else:
            self.__log.error(f"MQTT connection failed result code : [{str(rc)}] ");

    def on_disconnect(self, client: Any, user_data: Any, rc: Any) -> None:
        if rc != 0:
            self.__log.error(f"MQTT disconnection result code : [{str(rc)}] ");
            self.rerun();

    def on_message(self, client: Any, user_data: Any, msg: Any) -> None:
        pass;

    def publish(self, topic: str, payload: Any, qos: int) -> None:
        self.client.publish(topic=topic, payload=payload, qos=qos);

    def subscribe(self, topic: str, qos: int) -> None:
        self.__log.info(f"MQTT granted subscription\n\ttopic : {topic}\n\tqos : {qos}");
        self.client.subscribe(topic=topic, qos=qos);


__all__ = ["Client"];