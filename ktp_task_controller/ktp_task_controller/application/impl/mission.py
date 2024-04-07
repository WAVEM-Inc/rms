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
from ktp_task_controller.application.impl.mission import MissionProcessor;
from ktp_task_controller.application.impl.navigation import NavigationProcessor;
from ktp_task_controller.internal.utils import ros_message_dumps;
from ktp_task_controller.internal.utils import get_current_time;
from ktp_task_controller.internal.constants import (
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


class MissionProcessor:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__ublox_fix: NavSatFix = None;

        self.__mission: Mission = None;
    
        assign_mission_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            callback_group=assign_mission_service_cb_group,
            callback=self.assign_mission_service_cb,
            qos_profile=qos_profile_services_default
        );

        ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ublox_fix_subscription: Subscription = self.__node.create_subscription(
            topic=UBLOX_FIX_TOPIC_NAME,
            msg_type=NavSatFix,
            callback_group=ublox_fix_subscription_cb_group,
            callback=self.ublox_fix_subscription_cb,
            qos_profile=qos_profile_system_default
        );

        path_graph_path_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__path_graph_path_service_client: Client = self.__node.create_client(
            srv_name=PATH_GRAPH_PATH_SERVICE_NAME,
            srv_type=Path,
            callback_group=path_graph_path_service_client_cb_group,
            qos_profile=qos_profile_services_default
        );
    
        notify_mission_status_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_mission_status_publisher: Publisher = self.__node.create_publisher(
            topic=NOTIFY_MISSION_STATUS_TOPIC_NAME,
            msg_type=ServiceStatus,
            callback_group=notify_mission_status_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );

    def assign_mission_service_cb(self, request: AssignMission.Request, response: AssignMission.Response) -> AssignMission.Response:
        request_mission_json: str = ros_message_dumps(message=request.mission);
        self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} request\n{request_mission_json}");

        if self.__mission is not None:
            self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Mission has already scheduled...");
            response.result = False;
        else:
            self.__mission = message_conversion.populate_instance(json.loads(request_mission_json), Mission());

            global to_source_flag;
            to_source_flag = True;

            if self.__ublox_fix is None:
                self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} {UBLOX_FIX_TOPIC_NAME} is None...");
                request_mission_json = "";
                self.__mission = None;
                response.result = False;
                # self.error_report_publish(error_code="451");
            else:
                # self.command_navigation_with_path();
                response.result = True;

        return response;

    def notify_mission_status_publish(self, status: str) -> None:
        service_status: ServiceStatus = ServiceStatus();
        service_status.create_time = get_current_time();
        service_status.mission_code = self.__mission.mission_code;
        service_status.mission_id = self.__mission.mission_id;
        service_status.owner = self.__mission.owner;

        mission_task: MissionTask = self.__mission.task[0];
        service_status_task: ServiceStatusTask = ServiceStatusTask();
        service_status_task.task_id = mission_task.task_id;
        service_status_task.task_code = mission_task.task_code;
        service_status_task.status = status;
        service_status_task.seq = mission_task.seq;

        mission_task_data: MissionTaskData = mission_task.task_data;
        service_status_task_data: ServiceStatusTaskData = ServiceStatusTaskData();
        service_status_task_data.map_id = mission_task_data.map_id;
        service_status_task_data.source = mission_task_data.source;
        service_status_task_data.goal = mission_task_data.goal;
        service_status_task_data.lock_status = "0";

        service_status.task = [service_status_task];
        service_status_task.task_data = service_status_task_data;

        self.__log.info(
            f"{NOTIFY_MISSION_STATUS_TOPIC_NAME} Service Status\n{ros_message_dumps(message=service_status)}");
        self.__notify_mission_status_publisher.publish(msg=service_status);

    
__all__ = ["MissionProcessor"];