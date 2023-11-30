import os
import time
import socket
import paho.mqtt.client as mqtt

from typing import Any
from configparser import ConfigParser
from ..rms.common.service import ConfigService
from ..rms.common.service import UUIDService


""" Description

    This class for make logger in mqtt methods
    
    Attributes:
        - __info__: info logging flag of mqtt (str)
        - __warn__: warn logging flag of mqtt (str)
        - __error__: error logging flag of mqtt (str)
    
    Methods:
        - __init__: Constructor method for this class.
        - __get_current_time__: Get current unix time and formatt into str.
        - __log__: Log with logging level and color.
        - info: Log in info level.
        - warn: Log in warn level.
        - error: Log in error level.

"""
class Logger:
    __info: str = "[INFO]"
    __warn: str = "[WARN]"
    __error: str = "[ERROR]"

    def __init__(self) -> None:
        """ Description
        
        Constructor method for this class.
        
        Args:
            - self: This class' instance

        Returns:
            None
            
        Usage:
            __mqtt_logger__: mqtt_logger = mqtt_logger()
        """
        
        pass
    

    def __get_current_time(self) -> str:
        
        """ Description
        
        Get current unix time and formatt into str.
        
        Args:
            - self: This class' instance

        Returns:
            - formatted_time: formatted current unix time (str)
            
        Usage:
            formatted_current_time: str = self.__get_current_time__()
        
        """
        
        __current_time: float = time.time()
        __time_stamp: int = int(__current_time)
        __micro_seconds: int = int((__current_time - __time_stamp) * 1e9)
        formatted_time: str = "[{}.{}]".format(__time_stamp, __micro_seconds)

        return formatted_time


    def __log(self, start_color: str, level: str, message: str, end_color: str) -> None:
        
        """ Description
        
        Log with logging level and color.

        Args:
            - self: This class' instance
            - start_color: start color format (str)
            - level: logging level (str)
            - message: logging message (str)
            - end_color: end color format (str)

        Returns:
            None
            
        Usage:
            start_color: str = ""
            end_color: str = ""
            self.__log__(start_color, self.__info__, message, end_color)
        """
        
        __rclpy_node_name: str = 'rmde'
        __formatted_current_time: str = self.__get_current_time()
        print(
            start_color
            + "{} {} [{}]".format(level, __formatted_current_time, __rclpy_node_name)
            + ": "
            + message
            + end_color
        )


    def info(self, message: str) -> None:
        
        """ Description
        
        Log in info level
        
        Args:
            - self: This class' instance
            - message: logging message (str)

        Returns:
            None
            
        Usage:
            self.__mqtt_logger__.info("MQTT generated security key")
        
        """
        
        start_color: str = ""
        end_color: str = ""
        self.__log(start_color, self.__info, message, end_color)


    def warn(self, message: str) -> None:
        
        """ Description
        
        Log in warn level
        
        Args:
            - self: This class' instance
            - message: logging message (str)

        Returns:
            None
            
        Usage:
            self.__mqtt_logger__.warn("MQTT generated security key")
        
        """
        
        start_color: str = "\033[33m"
        end_color: str = "\033[0m"
        self.__log(start_color, self.__warn, message, end_color)


    def error(self, message: str) -> None:
        
        """ Description
        
        Log in error level
        
        Args:
            - self: This class' instance
            - message: logging message (str)

        Returns:
            None
            
        Usage:
            self.__mqtt_logger__.error("MQTT generated security key")
        
        """
        
        start_color: str = "\033[31m"
        end_color: str = "\033[0m"
        self.__log(start_color, self.__error, message, end_color)

""" Description

    This class for usage of paho.mqtt.client
    
    Attributes:
        - __broker_address__: The address of MQTT broker (str)
        - __broker_port__: port number of MQTT broker (str)
        - __client_name__: client name for register to MQTT broker (str)
        - __client_keep_alive__: limit time of this MQTT client (int)
        - __mqtt_logger__: The instance of mqtt_logger class (mqtt_logger)
        - client: The instance of mqtt.Client
    
    Methods:
        - __init__: Constructor method for this class. Register on Connect / Message methods and connect into MQTT broker
        - __on_connect__: Callback method that invoked when MQTT connected.
        - __generate_secret_key__: Generate MQTT secret key.
        - __create_cipher_suite__: Create MQTT cipher suite.
        - __encrypt_data__: Encode and encrypt MQTT message data.
        - __decrypt_data__: Decode and decrypt MQTT message data.
        - __on_message__: Callback method that invoked when MQTT subscription received message.
        - publish: MQTT publish into mqtt subscription with topic, payload.
        - subscribe: MQTT subscribe with topic.


"""

MQTT_CONFIG_FILE_PATH: str = 'mqtt.ini'
MQTT_CONFIG_FILE_BROKER_SECTION: str = 'broker'
MQTT_TRANSPORT_TYPE_WS: str = 'websockets'
MQTT_TRANSPORT_TYPE_TCP: str = 'tcp'
MQTT_WEBSOCKETS_PATH: str = '/ws'

class Client:

    def __init__(self) -> None:
        self.__mqtt_logger: Logger = Logger()
        self.client: mqtt.Client
        
        __script_directory: str = os.path.dirname(os.path.abspath(__file__))
        __config_file_path: str = MQTT_CONFIG_FILE_PATH
        __uuid_service: UUIDService = UUIDService()
        __config_service: ConfigService = ConfigService(__script_directory, __config_file_path)
        __config_parser: ConfigParser = __config_service.read()
            
        self.broker_address: str = __config_parser.get(MQTT_CONFIG_FILE_BROKER_SECTION, 'host')
        self.broker_port: int = int(__config_parser.get(MQTT_CONFIG_FILE_BROKER_SECTION, 'port'))
        self.__client_name: str = __uuid_service.generate_uuid()
        self.__client_keep_alive: int = int(__config_parser.get(MQTT_CONFIG_FILE_BROKER_SECTION, 'client_keep_alive'))
        self.__user_name: str = __config_parser.get(MQTT_CONFIG_FILE_BROKER_SECTION, 'user_name')
        self.__password: str = __config_parser.get(MQTT_CONFIG_FILE_BROKER_SECTION, 'password')
        self.__type: str = __config_parser.get(MQTT_CONFIG_FILE_BROKER_SECTION, 'type')
        self.is_connected: bool = False


    def check_broker_opened(self) -> bool:
        try:
            sock: socket = socket.create_connection(
                address = (self.broker_address, self.broker_port),
                timeout = None
            )
            sock.close()
            return True
        except socket.error:
            pass
        
        return False
        

    def connect(self) -> None:
        try:
            if self.__type == MQTT_TRANSPORT_TYPE_TCP:
                self.client = mqtt.Client(self.__client_name, clean_session = True, userdata = None, transport = MQTT_TRANSPORT_TYPE_TCP)
            elif self.__type == MQTT_TRANSPORT_TYPE_WS:
                self.client = mqtt.Client(self.__client_name, clean_session = True, userdata = None, transport = MQTT_TRANSPORT_TYPE_WS)
                self.client.ws_set_options(path = MQTT_WEBSOCKETS_PATH)
            else:
                return
            self.client.username_pw_set(self.__user_name, self.__password)
            
            self.client.on_connect = self.__on_connect
            self.client.on_disconnect = self.__on_disconnect
            self.client.on_message = self.__on_message
            self.client.connect(self.broker_address, self.broker_port, self.__client_keep_alive)

            if self.client.is_connected:
                self.__mqtt_logger.info(f'MQTT Client is connected to [{self.broker_address}:{self.broker_port}]')
                self.is_connected = self.client.is_connected
            else:
                self.__mqtt_logger.error('MQTT failed to connect')
                self.is_connected = self.client.is_connected
        except OSError as ose:
            self.__mqtt_logger.error(f'MQTT OSError : {ose}')
        except Exception as e:
            self.__mqtt_logger.error(f'MQTT Error : {e}')
    

    def run(self) -> None:
        if self.is_connected:
            self.__mqtt_logger.info('MQTT Client is running')
            self.client.loop_start()
        else:
            self.__mqtt_logger.error('MQTT Client is not connected to broker')
            return


    def rerun(self) -> None:
        self.client.disconnect()
        self.client.loop_stop()
        self.run()


    def __on_connect(self, client: Any, user_data: Any, flags: Any, rc: Any) -> None:
        
        """ Description
        
        Callback method that invoked when MQTT connected.
        
        Args:
            - self: This class' instance
            - client: MQTT client instance (Any)
            - user_data: MQTT user data (Any)
            - flags: MQTT connection flags (Any)
            - rc: MQTT connection result code (Any)
            
        Returns:
            None
            
        Usage:
            self.client.on_connect = self.__on_connect__
        """
        
        if rc == 0:
            self.__mqtt_logger.info(f'MQTT connection succeeded result code : [{str(rc)}]')
        else:
            self.__mqtt_logger.error(f'MQTT connection failed result code : [{str(rc)}] ')
            
    
    def __on_disconnect(self, client: Any, user_data: Any, rc: Any) -> None:
        if rc != 0:
            self.__mqtt_logger.error(f'MQTT disconnection result code : [{str(rc)}] ')
            self.rerun()
            

    def __on_message(self, client: Any, user_data: Any, msg: Any) -> None:
        
        """ Description
        
        Callback method that invoked when MQTT subscription received message.
        
        Args:
            - self: This class' instance
            - client: MQTT client instance (Any)
            - user_data: MQTT user data (Any)
            - msg: Received MQTT message data (Any)

        Returns:
            None
            
        Usage:
        
        """
    

    def publish(self, topic: str, payload: Any, qos: int) -> None:
        
        """ Description
        
        MQTT publish into mqtt subscription with topic, payload.
        
        Args:
            - self: This class' instance
            - topic: Target MQTT topic (str)
            - payload: Target MQTT payload (Any)

        Returns:
            None
            
        Usage:
        
        """
        
        self.client.publish(topic = topic, payload = payload, qos = qos)


    def subscribe(self, topic: str, qos: int) -> None:
        
        """ Description
        
        MQTT subscribe with topic.
        
        Args:
            - self: This class' instance
            - topic: Target MQTT topic (str)

        Returns:
            None
            
        Usage:
        
        """
        
        self.__mqtt_logger.info(f'MQTT granted subscription\n\ttopic : {topic}\n\tqos : {qos}')
        self.client.subscribe(topic = topic, qos = qos)


__all__ = ['mqtt_client']