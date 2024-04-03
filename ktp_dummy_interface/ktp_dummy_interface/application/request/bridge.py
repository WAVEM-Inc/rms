import json;
import paho.mqtt.client as mqtt;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.client import Client;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.srv import AssignControl;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.srv import AssignMission;
from ktp_data_msgs.msg import DetectedObject;
from ktp_dummy_interface.application.mqtt import Client;
from typing import Dict;
from typing import Any;

MQTT_CONTROL_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/control";
MQTT_MISSION_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/mission";
MQTT_DETECTED_OBJECT_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/detected_object";

ASSIGN_CONTROL_SERVICE_NAME: str = "/ktp_data_manager/assign/control";
ASSIGN_MISSION_SERVICE_NAME: str = "/ktp_data_manager/assign/mission";
DETECTED_OBJECT_TOPIC: str = "/rms/ktp/itf/detected_object";


class RequestBridge:

    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;

        self.mqtt_subscribe_for_request();

        assign_control_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_service_client: Client = self.__node.create_client(
            srv_name=ASSIGN_CONTROL_SERVICE_NAME,
            srv_type=AssignControl,
            qos_profile=qos_profile_services_default,
            callback_group=assign_control_service_client_cb_group
        );

        assign_mission_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service_client: Client = self.__node.create_client(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            qos_profile=qos_profile_services_default,
            callback_group=assign_mission_service_client_cb_group
        );

        detected_object_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__detected_object_publisher: Publisher = self.__node.create_publisher(
            topic=DETECTED_OBJECT_TOPIC,
            msg_type=DetectedObject,
            qos_profile=qos_profile_system_default,
            callback_group=detected_object_publisher_cb_group
        );

    def mqtt_subscribe_for_request(self) -> None:
        self.__mqtt_client.subscribe(topic=MQTT_CONTROL_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_CONTROL_REQUEST_TOPIC, callback=self.mqtt_control_request_cb);

        self.__mqtt_client.subscribe(topic=MQTT_MISSION_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_MISSION_REQUEST_TOPIC, callback=self.mqtt_mission_request_cb);

        self.__mqtt_client.subscribe(topic=MQTT_DETECTED_OBJECT_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_DETECTED_OBJECT_REQUEST_TOPIC, callback=self.mqtt_detected_object_cb);

    def mqtt_control_request_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            control: Control = message_conversion.populate_instance(msg=mqtt_json, inst=Control());
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=control), indent=4)}");

            self.assign_control_service_request(control=control);

        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{mqtt_topic} : {nefe}");
            return;

        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;

    def assign_control_service_request(self, control: Control) -> None:
        assign_control_request: AssignControl.Request = AssignControl.Request();
        assign_control_request.control = control;

        is_assign_control_service_server_ready: bool = self.__assign_control_service_client.wait_for_service(timeout_sec=0.8);

        if is_assign_control_service_server_ready:
            assign_control_response: AssignControl.Response = self.__assign_control_service_client.call(request=assign_control_request);
            self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} Response : {json.dumps(message_conversion.extract_values(inst=assign_control_response), indent=4)}");

            if assign_control_response is None:
                self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Response is None...");
                return;
        else:
            self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Service Server is Not Ready...");
            return;

    def mqtt_mission_request_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            mission: Mission = message_conversion.populate_instance(msg=mqtt_json, inst=Mission());
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=mission), indent=4)}");

            self.assign_mission_service_request(mission=mission);

        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{mqtt_topic} : {nefe}");
            return;

        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            return;

    def assign_mission_service_request(self, mission: Mission) -> None:
        assign_mission_request: AssignMission.Request = AssignMission.Request();
        assign_mission_request.mission = mission;

        is_assign_mission_service_server_ready: bool = self.__assign_mission_service_client.wait_for_service(timeout_sec=0.8);

        if is_assign_mission_service_server_ready:
            assign_mission_response: AssignControl.Response = self.__assign_mission_service_client.call(request=assign_mission_request);
            self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Response : {json.dumps(message_conversion.extract_values(inst=assign_mission_response), indent=4)}");

            if assign_mission_response is None:
                self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Response is None...");
                return;
        else:
            self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Service Server is Not Ready...");
            return;

    def mqtt_detected_object_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            detected_object: DetectedObject = message_conversion.populate_instance(msg=mqtt_json, inst=DetectedObject());
            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=detected_object), indent=4)}");

            self.detected_object_publish(detected_object=detected_object);

        except KeyError as ke:
            self.__log.error(f"Invalid JSON Key in MQTT {mqtt_topic} subscription callback: {ke}");
            return;

        except json.JSONDecodeError as jde:
            self.__log.error(f"Invalid JSON format in MQTT {mqtt_topic} subscription callback: {jde.msg}");
            return;

        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{mqtt_topic} : {nefe}");
            return;

        except Exception as e:
            self.__log.error(f"Exception in MQTT {mqtt_topic} subscription callback: {e}");
            raise;

    def detected_object_publish(self, detected_object: DetectedObject) -> None:
        self.__detected_object_publisher.publish(msg=detected_object);


__all__ = ["RequestBridge"];