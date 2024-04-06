import json;
import paho.mqtt.client as mqtt;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.task import Future;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Status;
from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_msgs.msg import ErrorReport;
from ktp_data_msgs.msg import ControlReport;
from ktp_data_msgs.msg import GraphList;
from ktp_data_msgs.msg import ObstacleDetect;
from ktp_data_msgs.msg import LiDARSignal;
from ktp_dummy_interface.application.mqtt import Client;
from ktp_dummy_interface.application.mqtt import filter_empty_values;
from typing import Dict;
from typing import Any;

MQTT_DEFAULT_QOS: int = 0;
MQTT_RBT_STATUS_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/rbt_status";
MQTT_SERVICE_STATUS_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/service_status";
MQTT_ERROR_REPORT_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/error_report";
MQTT_CONTROL_REPORT_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/control_report";
MQTT_GRAPH_LIST_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/graph_list";
MQTT_OBSTACLE_DETECT_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/obstacle_detect";
MQTT_LIDAR_SIGNAL_RESPONSE_TOPIC: str = "/rms/ktp/dummy/response/lidar_signal";

RBT_STATUS_TOPIC_NAME: str = "/rms/ktp/data/rbt_status";
SERVICE_STATUS_TOPIC_NAME: str = "/rms/ktp/data/service_status";
ERROR_REPORT_TOPIC_NAME: str = "/rms/ktp/data/error_report";
CONTROL_REPORT_TOPIC_NAME: str = "/rms/ktp/data/control_report";
GRAPH_LIST_TOPIC_NAME: str = "/rms/ktp/data/graph_list";
OBSTACLE_DETECT_TOPIC_NAME: str = "/rms/ktp/data/obstacle_detect";
LIDAR_SIGNAL_TOPIC_NAME: str = "/rms/ktp/data/lidar_signal";


class ResponseBridge:

    def __init__(self, node: Node, mqtt_client: Client) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mqtt_client: Client = mqtt_client;

        rbt_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rbt_status_subscription: Subscription = self.__node.create_subscription(
            topic=RBT_STATUS_TOPIC_NAME,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=rbt_status_subscription_cb_group,
            callback=self.rbt_status_subscription_cb
        );
    
        service_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__service_status_subscription_cb: Subscription = self.__node.create_subscription(
            topic=SERVICE_STATUS_TOPIC_NAME,
            msg_type=ServiceStatus,
            qos_profile=qos_profile_system_default,
            callback_group=service_status_subscription_cb_group,
            callback=self.service_status_subscription_cb
        );

        error_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_report_subscription: Subscription = self.__node.create_subscription(
            topic=ERROR_REPORT_TOPIC_NAME,
            msg_type=ErrorReport,
            qos_profile=qos_profile_system_default,
            callback_group=error_report_subscription_cb_group,
            callback=self.error_report_subscription_cb
        );
    
        control_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__control_report_subscription: Subscription = self.__node.create_subscription(
            topic=CONTROL_REPORT_TOPIC_NAME,
            msg_type=ControlReport,
            qos_profile=qos_profile_system_default,
            callback_group=control_report_subscription_cb_group,
            callback=self.control_report_subscription_cb
        );
    
        graph_list_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_subscription: Subscription = self.__node.create_subscription(
            topic=GRAPH_LIST_TOPIC_NAME,
            msg_type=GraphList,
            qos_profile=qos_profile_system_default,
            callback_group=graph_list_subscription_cb_group,
            callback=self.graph_list_subscription_cb
        );
    
        obstacle_detect_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_detect_subscription: Subscription = self.__node.create_subscription(
            topic=OBSTACLE_DETECT_TOPIC_NAME,
            msg_type=ObstacleDetect,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_detect_subscription_cb_group,
            callback=self.obstacle_detect_subscription_cb
        );
    
        lidar_signal_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__lidar_signal_subscription: Subscription = self.__node.create_subscription(
            topic=LIDAR_SIGNAL_TOPIC_NAME,
            msg_type=LiDARSignal,
            qos_profile=qos_profile_system_default,
            callback_group=lidar_signal_subscription_cb_group,
            callback=self.lidar_signal_subscription_cb
        );

    def mqtt_publish_after_filtering(self, topic: str, ros_callback_data: Any) -> None:
        try:
            properties: Any = message_conversion.extract_values(inst=ros_callback_data);
            properties_filtered: Any = filter_empty_values(properties);
            properties_filtered: str = json.dumps({k: v for k, v in properties_filtered.items() if v is not None});

            self.__mqtt_client.publish(topic=topic, payload=properties_filtered, qos=0);
        except message_conversion.NonexistentFieldException as nefe:
            self.__log.error(f"{topic} : {nefe}");
            return;
        except Exception as e:
            self.__log.error(f"{topic} : {e}");

    def rbt_status_subscription_cb(self, rbt_status_cb: Status) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_RBT_STATUS_RESPONSE_TOPIC, ros_callback_data=rbt_status_cb);

    def service_status_subscription_cb(self, service_status_cb: ServiceStatus) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_SERVICE_STATUS_RESPONSE_TOPIC, ros_callback_data=service_status_cb);
    
    def error_report_subscription_cb(self, error_report_cb: ErrorReport) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_ERROR_REPORT_RESPONSE_TOPIC, ros_callback_data=error_report_cb);

    def control_report_subscription_cb(self, control_report_cb: ControlReport) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_CONTROL_REPORT_RESPONSE_TOPIC, ros_callback_data=control_report_cb);

    def graph_list_subscription_cb(self, graph_list_cb: GraphList) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_GRAPH_LIST_RESPONSE_TOPIC, ros_callback_data=graph_list_cb);

    def obstacle_detect_subscription_cb(self, obstacle_detect_cb: ObstacleDetect) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_OBSTACLE_DETECT_RESPONSE_TOPIC, ros_callback_data=obstacle_detect_cb);
    
    def lidar_signal_subscription_cb(self, lidar_signal_cb: LiDARSignal) -> None:
        self.mqtt_publish_after_filtering(topic=MQTT_LIDAR_SIGNAL_RESPONSE_TOPIC, ros_callback_data=lidar_signal_cb);


__all__ = ["ResponseBridge"];