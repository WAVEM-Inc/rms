import socket;
import paho.mqtt.client as mqtt;
import paho.mqtt.publish as publish;
import paho.mqtt.subscribe as subscribe;

from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from typing import Any;


def filter_empty_values(data: Any) -> Any:
    if isinstance(data, dict):
        return {
            key: filter_empty_values(value)
            for key, value in data.items()
            if value is not None and filter_empty_values(value) is not None
        }
    elif isinstance(data, list):
        filtered_list = [
            filter_empty_values(item)
            for item in data
            if item is not None and filter_empty_values(item) is not None
        ]
        return [item for item in filtered_list if item]
    elif isinstance(data, str):
        return data if data.strip() != "" else None
    else:
        return data


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

    def connect(self) -> None:
        try:
            self.__log.info(
                f"MQTT Connect\n"
                f"host : {self.__host}\n"
                f"port : {self.__port}\n"
                f"type : {self.__type}\n");

            self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=self.__client_name, clean_session=True, userdata=None, transport=self.__type);
            self.client.ws_set_options(path="/mqtt");
                
            self.client.username_pw_set(self.__user_name, self.__password);

            self.client.on_connect = self.on_connect;
            self.client.on_message = self.on_message;
            self.client.connect(host=self.__host, port=self.__port, keepalive=self.__client_keep_alive);
        except OSError as ose:
            self.__log.error(f"MQTT OSError : {ose}");
        except Exception as e:
            self.__log.error(f"MQTT Error : {e}");

    def run(self) -> None:
        self.client.loop_forever(timeout=1.0, retry_first_connection=True);

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.__log.error(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            self.is_connected = True;
            self.__log.info(f"MQTT Succeeded to Connect");

    def on_disconnect(self, client: Any, user_data: Any, rc: Any) -> None:
        if rc != 0:
            self.__log.error(f"MQTT disconnection result code : [{str(rc)}] ");

    def on_message(self, client: Any, user_data: Any, msg: Any) -> None:
        pass;

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties) -> None:
        if reason_code_list[0].is_failure:
            self.__log.error(f"MQTT Broker rejected you subscription: {reason_code_list[0]}")
        else:
            self.__log.info(f"MQTT Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties) -> None:
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            self.__log.error("MQTT unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)");
        else:
            self.__log.info(f"MQTT Broker replied with failure: {reason_code_list[0]}");

    def publish(self, topic: str, payload: Any, qos: int) -> None:
        self.client.publish(topic=topic, payload=payload, qos=qos);

    def subscribe(self, topic: str, qos: int) -> None:
        self.__log.info(f"MQTT granted subscription\n\ttopic : {topic}\n\tqos : {qos}");
        self.client.subscribe(topic=topic, qos=qos);


__all__ = ["Client"];