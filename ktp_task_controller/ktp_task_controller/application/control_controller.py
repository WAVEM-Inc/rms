import json
import rclpy.action as rclpy_action;
from rclpy.action.client import ClientGoalHandle;
from rclpy.node import Node;
from rclpy.service import Service;
from rclpy.client import Client;
from rclpy.action.client import ActionClient;
from rclpy.subscription import Subscription;
from rclpy.publisher import Publisher;
from rclpy.timer import Timer;
from rclpy.task import Future;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Status;
from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_msgs.msg import ServiceStatusTask;
from ktp_data_msgs.msg import ServiceStatusTaskData;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.msg import MissionTask;
from ktp_data_msgs.msg import MissionTaskData;
from ktp_data_msgs.srv import AssignControl;
from path_graph_msgs.srv import Path;
from sensor_msgs.msg import NavSatFix;
from route_msgs.action import RouteToPose;
from typing import Any;
from ktp_task_controller.application.utils import ros_message_dumps;
from ktp_task_controller.application.utils import get_current_time;


NODE_NAME: str = "ktp_task_controller";
ASSIGN_CONTROL_SERVICE_NAME: str = f"/{NODE_NAME}/assign/assign";
PATH_GRAPH_PATH_SERVICE_NAME: str = "/path_graph_msgs/path";
UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
NOTIFY_MISSION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/mission/status";
NOTIFY_NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/navigation/status";

class ControlController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        assign_control_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_service: Service = self.__node.create_service(
            srv_name=ASSIGN_CONTROL_SERVICE_NAME,
            srv_type=AssignControl,
            callback_group=assign_control_service_cb_group,
            callback=self.assign_control_service_cb
        );
        
    def assign_control_service_cb(self, request: AssignControl.Request, response: AssignControl.Response) -> AssignControl.Response:
        pass;