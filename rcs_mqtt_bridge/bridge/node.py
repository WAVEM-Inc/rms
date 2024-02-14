import json
import importlib
import paho.mqtt.client as mqtt

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosbridge_library.internal import message_conversion

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
                mqtt_topic: str = mqtt_message.topic;
                mqtt_decoded_payload: str = mqtt_message.payload.decode();
                mqtt_json: Any = json.loads(mqtt_message.payload);
                
                self.get_logger().info(f'[{mqtt_topic}] subscription cb json [{mqtt_decoded_payload}]');

                for ros_connections in mqtt_json:
                    self.get_logger().info(f'mqtt_json ros_connections: [{ros_connections}]');

                    ros_connection_type: str = ros_connections["type"];
                    ros_connection_name: str = ros_connections["name"];
                    ros_connection_message_type: str = ros_connections["message_type"];

                    if ros_connection_type == "sub":
                        self.__create_rcl_subscription(topic=ros_connection_name, message_type=ros_connection_message_type);
                    elif ros_connection_type == "pub":
                        self.__create_rcl_publisher(topic=ros_connection_name, message_type=ros_connection_message_type);
                    elif ros_connection_type == "call":
                        pass;
                    elif ros_connection_type == "goal":
                        pass;
                
            except KeyError as ke:
                self.get_logger().error(f'Invalid JSON Key in MQTT [{mqtt_topic}] subscription callback: {ke}');

            except json.JSONDecodeError as jde:
                self.get_logger().error(f'Invalid JSON format in MQTT [{mqtt_topic}] subscription callback: {jde.msg}');

            except Exception as e:
                self.get_logger().error(f'Exception in MQTT [{mqtt_topic}] subscription callback: {e}');
                raise
            
        self.__mqtt_client.subscribe(topic=self.__ros_message_init_topic);
        self.__mqtt_client.client.message_callback_add(sub=self.__ros_message_init_topic, callback=__ros_message_init_sub_cb);
    
    def __extract_ros_messages(self, message_type: str) -> None:
        message_type_split: list[str] = message_type.split("/", 3);
        message_package_module_name: str = f'{message_type_split[0]}.{message_type_split[1]}';
        message_class_name: str = f'{message_type_split[2]}';

        message_package_module: Any = importlib.import_module(f'{message_type_split[0]}.{message_type_split[1]}');

        self.get_logger().info(f'import_module : {message_package_module}');

        message_class: Any = getattr(message_package_module, message_class_name);
        
        self.get_logger().info(f'message_class : {message_class}');

        return message_class;

    def __lookup_ros_messages(self, module_name: str, module_class_name: str) -> Any:
        self.get_logger().info(f'lookup object module_name : {module_name}');
        self.get_logger().info(f'lookup object module_class_name : {module_class_name}');

        message_path: Any = importlib.import_module(module_name, self.get_name());
        message_object: Any = getattr(message_path, module_class_name);

        return message_object;
    
    def __create_rcl_subscription(self, topic: str, message_type: str) -> None:
        self.get_logger().info(f'create_rcl_subscription topic : [{topic}], message_type : [{message_type}]');

        def __rcl_subscription_cb(cb_message: Any) -> None:
             mqtt_serialized_message: str = json.dumps(message_conversion.extract_values(cb_message));
             mqtt_topic_name: str = topic;
             self.__mqtt_client.publish(topic=mqtt_topic_name, payload=mqtt_serialized_message);

        self.create_subscription(
            msg_type=self.__extract_ros_messages(message_type),
            topic=topic,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=__rcl_subscription_cb
        );
    
    def __create_rcl_publisher(self, topic: str, message_type: str) -> None:
        rcl_publisher: Publisher = self.create_publisher(
            msg_type=self.__extract_ros_messages(message_type),
            topic=topic,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup()
        );

        self.get_logger().info(f'create_rcl_publisher topic : [{topic}], message_type : [{message_type}]');

        def __publisher_mqtt_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage):
            try:
                mqtt_topic: str = mqtt_message.topic;
                mqtt_decoded_payload: str = mqtt_message.payload.decode();
                mqtt_json: Any = json.loads(mqtt_message.payload);

                self.get_logger().info(f'[{mqtt_topic}] publisher mqtt_cb json [{mqtt_decoded_payload}]');
                
                ros_message_type_split: list[str] = message_type.split("/", 3);
                ros_message_obj: Any = self.__lookup_ros_messages(f"{ros_message_type_split[0]}.{ros_message_type_split[1]}", f"{ros_message_type_split[2]}");
                ros_message_class: Any = message_conversion.populate_instance(mqtt_json["data"], ros_message_obj());
                
                if mqtt_json["mode"]:
                    rcl_publisher.publish(ros_message_class);
                else:
                    pass;
            except KeyError as ke:
                self.get_logger().error(f'Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}')

            except json.JSONDecodeError as jde:
                self.get_logger().error(f'Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}')

            except Exception as e:
                self.get_logger().error(f'Exception in MQTT {mqtt_topic} subscription callback: {e}')
                raise;

        self.__mqtt_client.subscribe(topic=topic);
        self.__mqtt_client.client.message_callback_add(sub=topic, callback=__publisher_mqtt_subscription_cb);
