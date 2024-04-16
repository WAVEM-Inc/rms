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
from std_msgs.msg import String;
from obstacle_msgs.msg import Status;
from can_msgs.msg import Emergency;
from ktp_dummy_interface.application.mqtt import Client;
from typing import Dict;
from typing import Any;

MQTT_CONTROL_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/control";
MQTT_MISSION_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/mission";
MQTT_DETECTED_OBJECT_REQUEST_TOPIC: str = "/rms/ktp/dummy/request/detected_object";
MQTT_ERROR_STATUS_TOPIC: str = "/rms/ktp/dummy/request/error_status";
MQTT_OBSTACLE_STATUS_TOPIC: str = "/rms/ktp/dummy/request/obstacle/status";
MQTT_DRIVE_OBSTACLE_COOPERATIVE_TOPIC: str = "/rms/ktp/dummy/request/obstacle/cooperative";
MQTT_CAN_EMERGENCY_STOP_TOPIC: str = "/rms/ktp/dummy/request/can/emergency";

ASSIGN_CONTROL_SERVICE_NAME: str = "/ktp_data_manager/assign/control";
ASSIGN_MISSION_SERVICE_NAME: str = "/ktp_data_manager/assign/mission";
DETECTED_OBJECT_TOPIC: str = "/rms/ktp/itf/detected_object";
ERROR_STATUS_TOPIC: str = "/rms/ktp/data/notify/error/status";
OBSTACLE_EVENT_TOPIC: str = "/drive/obstacle/event";
DRIVE_OBSTACLE_COOPERATIVE_TOPIC: str = "/drive/obstacle/cooperative";
CAN_EMERGENCY_STOP_TOPIC: str = "/drive/can/emergency";


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

        error_status_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_status_publisher: Publisher = self.__node.create_publisher(
            topic=ERROR_STATUS_TOPIC,
            msg_type=String,
            qos_profile=qos_profile_system_default,
            callback_group=error_status_publisher_cb_group
        );

        obstacle_event_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_event_publisher: Publisher = self.__node.create_publisher(
            topic=OBSTACLE_EVENT_TOPIC,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_event_publisher_cb_group
        );

        drive_obstacle_cooperative_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__drive_obstacle_cooperative_publisher: Publisher = self.__node.create_publisher(
            topic=DRIVE_OBSTACLE_COOPERATIVE_TOPIC,
            msg_type=String,
            qos_profile=qos_profile_system_default,
            callback_group=drive_obstacle_cooperative_publisher_cb_group
        );
        
        can_emergency_stop_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__can_emergency_stop_publisher: Publisher = self.__node.create_publisher(
            topic=CAN_EMERGENCY_STOP_TOPIC,
            msg_type=Emergency,
            qos_profile=qos_profile_system_default,
            callback_group=can_emergency_stop_cb_group
        );

    def mqtt_subscribe_for_request(self) -> None:
        self.__mqtt_client.subscribe(topic=MQTT_CONTROL_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_CONTROL_REQUEST_TOPIC, callback=self.mqtt_control_request_cb);

        self.__mqtt_client.subscribe(topic=MQTT_MISSION_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_MISSION_REQUEST_TOPIC, callback=self.mqtt_mission_request_cb);

        self.__mqtt_client.subscribe(topic=MQTT_DETECTED_OBJECT_REQUEST_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_DETECTED_OBJECT_REQUEST_TOPIC, callback=self.mqtt_detected_object_cb);

        self.__mqtt_client.subscribe(topic=MQTT_ERROR_STATUS_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_ERROR_STATUS_TOPIC, callback=self.mqtt_error_status_cb);

        self.__mqtt_client.subscribe(topic=MQTT_OBSTACLE_STATUS_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_OBSTACLE_STATUS_TOPIC, callback=self.mqtt_obstacle_event_cb);

        self.__mqtt_client.subscribe(topic=MQTT_DRIVE_OBSTACLE_COOPERATIVE_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_DRIVE_OBSTACLE_COOPERATIVE_TOPIC, callback=self.mqtt_drive_obstacle_cooperative_cb);
        
        self.__mqtt_client.subscribe(topic=MQTT_CAN_EMERGENCY_STOP_TOPIC, qos=0);
        self.__mqtt_client.client.message_callback_add(sub=MQTT_CAN_EMERGENCY_STOP_TOPIC, callback=self.mqtt_can_emergency_cb);

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
            return;

    def detected_object_publish(self, detected_object: DetectedObject) -> None:
        self.__detected_object_publisher.publish(msg=detected_object);

    def mqtt_error_status_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            error_status: String = message_conversion.populate_instance(msg=mqtt_json, inst=String());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=error_status), indent=4)}");

            self.error_status_publish(error_status=error_status);
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

    def error_status_publish(self, error_status: String) -> None:
        self.__error_status_publisher.publish(msg=error_status);

    def mqtt_obstacle_event_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            obstacle_status: Status = message_conversion.populate_instance(msg=mqtt_json, inst=Status());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=obstacle_status), indent=4)}");

            self.obstacle_event_publish(obstacle_status=obstacle_status);
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

    def obstacle_event_publish(self, obstacle_status: Status) -> None:
        self.__obstacle_event_publisher.publish(msg=obstacle_status);

    def mqtt_drive_obstacle_cooperative_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            obstacle_cooperative: String = message_conversion.populate_instance(msg=mqtt_json, inst=String());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=obstacle_cooperative), indent=4)}");

            self.drive_obstacle_cooperative_publish(obstacle_cooperative=obstacle_cooperative);
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

    def drive_obstacle_cooperative_publish(self, obstacle_cooperative: String) -> None:
        self.__drive_obstacle_cooperative_publisher.publish(msg=obstacle_cooperative);
        
    def can_emergency_publish(self, emergency: Emergency) -> None:
        self.__can_emergency_stop_publisher.publish(msg=emergency);
        
    def mqtt_can_emergency_cb(self, mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
        try:
            mqtt_topic: str = mqtt_message.topic;
            mqtt_decoded_payload: str = mqtt_message.payload.decode();
            mqtt_json: Any = json.loads(mqtt_message.payload);

            emergency: String = message_conversion.populate_instance(msg=mqtt_json, inst=Emergency());

            self.__log.info(f"{mqtt_topic} cb\n{json.dumps(obj=message_conversion.extract_values(inst=emergency), indent=4)}");

            self.can_emergency_publish(emergency=emergency);
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


__all__ = ["RequestBridge"];