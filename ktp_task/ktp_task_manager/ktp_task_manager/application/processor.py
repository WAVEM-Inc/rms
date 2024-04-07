import json;
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
from ktp_data_msgs.srv import AssignMission;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.msg import ControlReport;
from ktp_data_msgs.srv import AssignControl;
from ktp_data_msgs.msg import GraphList;
from ktp_data_msgs.msg import ControlReportDataGraphList;
from ktp_data_msgs.msg import ObstacleDetect;
from std_msgs.msg import String;
from path_graph_msgs.srv import Path;
from path_graph_msgs.srv import Graph;
from sensor_msgs.msg import NavSatFix;
from route_msgs.action import RouteToPose;
from action_msgs.msg import GoalStatus;
import route_msgs.msg as route;
import obstacle_msgs.msg as obstacle;
from typing import Any;
from typing import Tuple;
from ktp_task_manager.internal.utils import ros_message_dumps;
from ktp_task_manager.internal.utils import get_current_time;
from ktp_task_manager.internal.constants import (
    ASSIGN_MISSION_SERVICE_NAME,
    PATH_GRAPH_PATH_SERVICE_NAME,
    PATH_GRAPH_GRAPH_SERVICE_NAME,
    UBLOX_FIX_TOPIC_NAME,
    ROUTE_TO_POSE_ACTION_NAME,
    NOTIFY_MISSION_STATUS_TOPIC_NAME,
    NOTIFY_NAVIGATION_STATUS_TOPIC_NAME,
    ERROR_REPORT_TOPIC_NAME,
    ASSIGN_CONTROL_SERVICE_NAME,
    NOTIFY_CONTROL_REPORT_TOPIC_NAME,
    NOTIFY_OBSTACLE_DETECT_TOPIC_NAME,
    PATH_GRAPH_GRAPH_SERVICE_NAME,
    GRAPH_LIST_TOPIC,
    DRIVE_OBSTACLE_TOPIC,
    CONTROL_CODE_STOP,
    CONTROL_CODE_RELEASE,
    CONTROL_MS_CANCEL,
    CONTROL_CODE_MOVE_TO_DEST,
    CONTROL_CODE_MS_COMPLETE,
    CONTROL_CODE_GRAPH_SYNC,
    DRIVE_STATUS_WAIT,
    DRIVE_STATUS_ON_DRIVE,
    DRIVE_STATUS_DRIVE_FINISHED,
    DRIVE_STATUS_CANCELLED,
    DRIVE_STATUS_OBJECT_DETECTED,
    DRIVE_STATUS_DRIVE_FAILED,
    DRIVE_STATUS_MISSION_IMPOSSIBLE
);
from ktp_task_manager.application.impl.path import PathProcessor;


class Processor:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__device_id: str = self.__node.get_parameter(name="device_id").get_parameter_value().string_value;
        self.__map_id: str = self.__node.get_parameter(name="map_id").get_parameter_value().string_value;

        path_processor: PathProcessor = PathProcessor(node=self.__node);


__all__ = ["Processor"];