import json;
import importlib;
import paho.mqtt.client as mqtt;
from rclpy.node import Node
from rclpy.publisher import Publisher;
from rclpy.client import Client;
from rclpy.task import Future;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rosbridge_library.internal import message_conversion;
from rosbridge_library.internal.ros_loader import get_service_class;
from typing import Any;
from typing import Dict;
from rcs_mqtt_bridge.mqtt.client import Client;

NODE_NAME: str = "rcs_mqtt_bridge";

class RcsMQTTBridge(Node):
    
    def __init__(self) -> None:
        super().__init__(NODE_NAME);
        
        self.get_logger().info(f"{NODE_NAME} created");
        self.__declare_mqtt_parameters();
        
        self.__mqtt_client: Client = Client(self);
        
        self.__ros_message_init_topic: str = "ros_message_init";
        
        self.__ros_message_init_sub();
        
        self.__ros_established_topics: list[Any] = [];
        self.__ros_established_services: list[Any] = [];
        
    def __declare_mqtt_parameters(self) -> None:
        self.declare_parameter(name="host", value="");
        self.declare_parameter(name="port", value=0);
    
    def __ros_message_init_sub(self) -> None:       
        def __ros_message_init_sub_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic;
                mqtt_decoded_payload: str = mqtt_message.payload.decode();
                mqtt_json: Any = json.loads(mqtt_message.payload);
                
                self.get_logger().info(f"[{mqtt_topic}] subscription cb json [{mqtt_decoded_payload}]");

                for ros_connections in mqtt_json:
                    self.get_logger().info(f"mqtt_json ros_connections: [{ros_connections}]");

                    ros_connection_type: str = ros_connections["type"];
                    ros_connection_name: str = ros_connections["name"];
                    ros_connection_message_type: str = ros_connections["message_type"];

                    if ros_connection_type == "sub":
                        self.__create_rcl_subscription(topic=ros_connection_name, message_type=ros_connection_message_type);
                    elif ros_connection_type == "pub":
                        self.__create_rcl_publisher(topic=ros_connection_name, message_type=ros_connection_message_type);
                    elif ros_connection_type == "call":
                        self.__create_rcl_service_client(service_name=ros_connection_name, service_type=ros_connection_message_type);
                    else:
                        pass;
            except KeyError as ke:
                self.get_logger().error(f"Invalid JSON Key in MQTT [{mqtt_topic}] subscription callback: {ke}");
                return;
            except json.JSONDecodeError as jde:
                self.get_logger().error(f"Invalid JSON format in MQTT [{mqtt_topic}] subscription callback: {jde.msg}");
                return;
            except Exception as e:
                self.get_logger().error(f"Exception in MQTT [{mqtt_topic}] subscription callback: {e}");
                return;
            
        self.__mqtt_client.subscribe(topic=self.__ros_message_init_topic);
        self.__mqtt_client.client.message_callback_add(sub=self.__ros_message_init_topic, callback=__ros_message_init_sub_cb);
    
    def __extract_ros_message_class(self, message_type: str) -> None:
        message_type_split: list[str] = message_type.split("/", 3);
        message_package_module_name: str = f"{message_type_split[0]}.{message_type_split[1]}";
        message_class_name: str = f"{message_type_split[2]}";

        message_package_module: Any = importlib.import_module(f"{message_type_split[0]}.{message_type_split[1]}");

        self.get_logger().info(f"import_module : {message_package_module}");

        message_class: Any = getattr(message_package_module, message_class_name);
        
        self.get_logger().info(f"message_class : {message_class}");

        return message_class;

    def __lookup_ros_messages(self, module_name: str, module_class_name: str) -> Any:
        self.get_logger().info(f"lookup object module_name : {module_name}");
        self.get_logger().info(f"lookup object module_class_name : {module_class_name}");

        message_path: Any = importlib.import_module(module_name, self.get_name());
        message_object: Any = getattr(message_path, module_class_name);

        return message_object;
    
    def __create_rcl_subscription(self, topic: str, message_type: str) -> None:
        self.get_logger().info(f"create_rcl_subscription topic : [{topic}], message_type : [{message_type}]");
        
        for ros_topic in self.__ros_established_topics:
            if topic == ros_topic["topic"]:
                self.get_logger().info(f"{topic} is already established...");
                return;

        def __rcl_subscription_cb(cb_message: Any) -> None:
             mqtt_serialized_message: str = json.dumps(message_conversion.extract_values(cb_message));
             mqtt_topic_name: str = topic;
             self.__mqtt_client.publish(topic=mqtt_topic_name, payload=mqtt_serialized_message);

        self.create_subscription(
            msg_type=self.__extract_ros_message_class(message_type),
            topic=topic,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=__rcl_subscription_cb
        );
        
        topic_info: Any = {
            "topic": topic,
            "message_type": message_type
        };
        
        self.__ros_established_topics.append(topic_info);
    
    def __create_rcl_publisher(self, topic: str, message_type: str) -> None:
        # for ros_topic in self.__ros_established_topics:
        #     if topic == ros_topic["topic"]:
        #         self.get_logger().info(f"{topic} is already established...");
        #         return;
            
        rcl_publisher: Publisher = self.create_publisher(
            msg_type=self.__extract_ros_message_class(message_type),
            topic=topic,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup()
        );
        
        topic_info: Any = {
            "topic": topic,
            "message_type": message_type
        };
        
        self.__ros_established_topics.append(topic_info);

        self.get_logger().info(f"create_rcl_publisher topic : [{topic}], message_type : [{message_type}]");

        def __publisher_mqtt_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic;
                mqtt_decoded_payload: str = mqtt_message.payload.decode();
                mqtt_json: Any = json.loads(mqtt_message.payload);

                self.get_logger().info(f"[{mqtt_topic}] publisher mqtt_cb json [{mqtt_decoded_payload}]");
                
                ros_message_type_split: list[str] = message_type.split("/", 3);
                ros_message_obj: Any = self.__lookup_ros_messages(f"{ros_message_type_split[0]}.{ros_message_type_split[1]}", f"{ros_message_type_split[2]}");
                ros_message_class: Any = message_conversion.populate_instance(mqtt_json["data"], ros_message_obj());
                
                if mqtt_json["mode"] == "pub":
                    rcl_publisher.publish(ros_message_class);
                else:
                    return;
            except KeyError as ke:
                self.get_logger().error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
                return;
            except json.JSONDecodeError as jde:
                self.get_logger().error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
                return;
            except Exception as e:
                self.get_logger().error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
                return;

        self.__mqtt_client.subscribe(topic=topic);
        self.__mqtt_client.client.message_callback_add(sub=topic, callback=__publisher_mqtt_subscription_cb);

    def __create_rcl_service_client(self, service_name: str, service_type: str) -> None:
        for ros_service in self.__ros_established_services:
            if service_name == ros_service["service_name"]:
                self.get_logger().info(f"{service_name} is already established...");
                return;
            
        rcl_client: Client = self.create_client(
            srv_type=self.__extract_ros_message_class(service_type),
            srv_name=service_name,
            qos_profile=qos_profile_services_default,
            callback_group=MutuallyExclusiveCallbackGroup()
        );
        
        service_info: Any = {
            "service_name": service_name,
            "service_type": service_type
        };
        
        self.__ros_established_services.append(service_info);

        self.get_logger().info(f"cretae_rcl_service_client name : [{service_name}], service_type : [{service_type}]");

        def __service_client_mqtt_subscription_cb(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            try:
                mqtt_topic: str = mqtt_message.topic;
                mqtt_decoded_payload: str = mqtt_message.payload.decode();
                mqtt_json: Any = json.loads(mqtt_message.payload);

                self.get_logger().info(f"[{mqtt_topic}] service_client mqtt_cb json [{mqtt_decoded_payload}]");

                ros_message_type_split: list[str] = service_type.split("/", 3);
                ros_message_obj: Any = self.__lookup_ros_messages(f"{ros_message_type_split[0]}.{ros_message_type_split[1]}", f"{ros_message_type_split[2]}");

                ros_service_class: Any = get_service_class(typestring=service_type);
                ros_message_class: Any = message_conversion.populate_instance(mqtt_json["data"], ros_service_class.Request());

                self.get_logger().info(f"request service message : [{ros_message_class}]");
                
                if mqtt_json["mode"] == "call":
                    is_rcl_service_ready: bool = rcl_client.wait_for_service(timeout_sec=1.0);
                    self.get_logger().info(f"is service server ready : [{is_rcl_service_ready}]");

                    if not is_rcl_service_ready:
                        self.get_logger().error(f"service server is not ready...aborting");
                        return;
                    else:
                        request_future: Future = rcl_client.call_async(ros_service_class);
                        self.get_logger().info(f"request_future : [{request_future}]");

                        request_future_done: bool = request_future.done();
                        self.get_logger().info(f"request_future_done : [{request_future_done}]");

                        if request_future_done:
                            request_future_result: Any = request_future_done.result();
                            self.get_logger().info(f"request_future_result : [{request_future_result}]");

                            request_future_result_json: Any = json.dumps(message_conversion.extract_values(request_future_result));
                            mqtt_response_topic: str = f"{service_name}/response";
                            self.__mqtt_client.publish(topic=mqtt_response_topic, payload=request_future_result_json);
                else:
                    pass;
            except KeyError as ke:
                self.get_logger().error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
                return;
            except json.JSONDecodeError as jde:
                self.get_logger().error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
                return;
            except Exception as e:
                self.get_logger().error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
                return;
        
        mqtt_request_topic: str = f"{service_name}/request";
        self.__mqtt_client.subscribe(topic=mqtt_request_topic);
        self.__mqtt_client.client.message_callback_add(sub=mqtt_request_topic, callback=__service_client_mqtt_subscription_cb);


__all__ = ["rcs_mqtt_bridge.bridge.node"];
