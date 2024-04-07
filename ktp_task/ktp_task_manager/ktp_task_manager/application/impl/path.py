import json;
from ktp_task_controller.ktp_task_controller.internal.constants import NAVIGATION_STATUS_TOPIC_NAME
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
    PATH_GRAPH_PATH_SERVICE_NAME,
    READY_TO_NAVIGATION_TOPIC_NAME,
    UBLOX_FIX_TOPIC_NAME,
    CONVERT_TASK_TO_PATH_SERVICE_NAME,
    LOOK_UP_PATH_SERVICE_NAME,
    NAVIGATION_STATUS_TOPIC_NAME,
    NAVIGATION_STATUS_READY_TO_MOVE
);
from ktp_task_msgs.srv import ConvertTaskToPath;
from ktp_task_msgs.srv import LookUpPath;


class PathProcessor:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.__path_waiting_place_to_source: str | Any = self.__node.get_parameter(name="path_waiting_place_to_source").get_parameter_value().string_value;
        self.__path_source_to_goal: str | Any = self.__node.get_parameter(name="path_source_to_goal").get_parameter_value().string_value;
        self.__path_goal_to_waiting_place : str | Any = self.__node.get_parameter(name="path_goal_to_waiting_place").get_parameter_value().string_value;
        
        self.__path_list: list[route.Path] = [];
        self.__ublox_fix: NavSatFix = None;

        convert_task_to_path_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__convert_task_to_path_service: Service = self.__node.create_service(
            srv_name=CONVERT_TASK_TO_PATH_SERVICE_NAME,
            srv_type=ConvertTaskToPath,
            callback_group=convert_task_to_path_service_cb_group,
            callback=self.convert_task_to_path_service_cb,
            qos_profile=qos_profile_services_default
        );
        
        look_up_path_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__look_up_path_service: Service = self.__node.create_service(
            srv_name=LOOK_UP_PATH_SERVICE_NAME,
            srv_type=LookUpPath,
            callback_group=look_up_path_service_cb_group,
            callback=self.look_up_path_service_cb,
            qos_profile=qos_profile_services_default
        );

        path_graph_path_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__path_graph_path_service_client: Client = self.__node.create_client(
            srv_name=PATH_GRAPH_PATH_SERVICE_NAME,
            srv_type=Path,
            callback_group=path_graph_path_service_client_cb_group,
            qos_profile=qos_profile_services_default
        );
        
        ready_to_move_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ready_to_move_publisher: Publisher = self.__node.create_publisher(
            topic=READY_TO_NAVIGATION_TOPIC_NAME,
            msg_type=Status,
            callback_group=ready_to_move_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );
        
        navigation_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__navigation_status_subscription: Subscription = self.__node.create_subscription(
            topic=NAVIGATION_STATUS_TOPIC_NAME,
            msg_type=Status,
            callback_group=navigation_status_subscription_cb_group,
            callback=self.navigation_status_subscription_cb,
            qos_profile=qos_profile_system_default
        );
        
        ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ublox_fix_subscription: Subscription = self.__node.create_subscription(
            topic=UBLOX_FIX_TOPIC_NAME,
            msg_type=NavSatFix,
            callback_group=ublox_fix_subscription_cb_group,
            callback=self.ublox_fix_subscription_cb,
            qos_profile=qos_profile_system_default
        );
        
    def set_up_path_request(self, task_request: MissionTask) -> list[Path.Request]:
        path_request_list: list[Path.Request] = [];
        
        """
        대기 장소 -> 상차지(출발지)
        - position : 현재 GPS
        - start_node : null
        - end_node : task.task_data.source
        """
        waiting_to_source_path_request: Path.Request = Path.Request();
        waiting_to_source_path_request.position.longitude = 0.0;
        waiting_to_source_path_request.position.latitude = 0.0;
        waiting_to_source_path_request.start_node = "";
        waiting_to_source_path_request.end_node = task_request.task_data.source;
        
        path_request_list.append(waiting_to_source_path_request);
        
        """
        상차지(출발지) -> 하차지
        - position : 현재 GPS
        - start_node : task.task_data.source
        - end_node : task.task_data.goal[0]
        """
        source_to_goal_path_request: Path.Request = Path.Request();
        source_to_goal_path_request.position.longitude = self.__ublox_fix.longitude;
        source_to_goal_path_request.position.latitude = self.__ublox_fix.latitude;
        source_to_goal_path_request.start_node = task_request.task_data.source;
        source_to_goal_path_request.end_node = task_request.task_data.goal[0];
        
        path_request_list.append(source_to_goal_path_request);
        
        """
        하차지 -> 대기 장소
        - position : 현재 GPS
        - start_node : task.task_data.goal[0]
        - end_node : null
        """
        goal_to_waiting_place_path_request: Path.Request = Path.Request();
        goal_to_waiting_place_path_request.position.longitude = self.__ublox_fix.longitude;
        goal_to_waiting_place_path_request.position.latitude = self.__ublox_fix.latitude;
        goal_to_waiting_place_path_request.start_node = task_request.task_data.goal[0];
        goal_to_waiting_place_path_request.end_node = "";
        
        path_request_list.append(goal_to_waiting_place_path_request);
        
        return path_request_list;
        
    def convert_task_to_path_service_cb(self, request: ConvertTaskToPath.Request, response: ConvertTaskToPath.Response) -> ConvertTaskToPath.Response:
        task_request: MissionTask = request.task;
        path_request_list: list[Path.Request] = self.set_up_path_request(task_request=task_request);
        path_response_list: list[route.Path] = [];
        
        if self.__ublox_fix is not None:
            for path_index, path_request in enumerate(iterable=path_request_list):
                self.__log.info(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} path_request[{path_index}] : {ros_message_dumps(message=path_request)}");
                
                is_path_service_server_ready: bool = self.__path_graph_path_service_client.wait_for_service(timeout_sec=0.75);
                
                if is_path_service_server_ready:
                    path_response: Path.Response = self.__path_graph_path_service_client.call(request=path_request);
                    self.__log.info(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} {PATH_GRAPH_PATH_SERVICE_NAME} path_response[{path_index}] : {ros_message_dumps(message=path_response)}");
                    
                    route_path: route.Path = path_response.path;
                    path_response_list.append(route_path);
                    self.__path_list = path_response_list;
                    
                    if not self.__path_list:
                        self.__log.error(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} conversion failed...");
                        response.result = False;
                    else:
                        self.__log.info(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} conversion succeeded");
                        response.result = True;
                        
                        ready_to_move_trigger: Status = Status();
                        ready_to_move_trigger.drive_status = NAVIGATION_STATUS_READY_TO_MOVE;
                        self.__ready_to_move_publisher.publish(msg=ready_to_move_trigger);
                else:
                    self.__log.error(message=f"");
                    response.result = False;
        else:
            response.result = False;
        
        return response;
    
    def look_up_path_service_cb(self, request: LookUpPath.Request, response: LookUpPath.Response) -> LookUpPath.Response:
        path_key: str = request.path_key;
        self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME} path_key : {path_key}");
        
        path_index: int = 0;
        
        if path_key == self.__path_waiting_place_to_source:
            path_index = 0;
            self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME}[{path_index}] W/P -> SOURCE");
        elif path_key == self.__path_source_to_goal:
            path_index = 1;
            self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME}[{path_index}] SOURCE -> GOAL");
        elif path_key == self.__path_goal_to_waiting_place:
            path_index = 2;
            self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME}[{path_index}] GOAL -> W/P");
        else:
            path_index = 0;
            self.__log.error(message=f"{LOOK_UP_PATH_SERVICE_NAME} Unknown path_key aborting...");
            response.path = None;
        
        response.path = self.__path_list[path_index];
        self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME} LookUp Path\n{ros_message_dumps(message=response.path)}");
        
        return response;
        
    def path_graph_path_service_request(self, path_request: Path.Request) -> route.Path:
        self.__log.info(message=f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path Request is ready\n{ros_message_dumps(message=path_request)}");

        is_path_graph_path_service_server_ready: bool = self.__path_graph_path_service_client.wait_for_service(timeout_sec=1.0);

        if is_path_graph_path_service_server_ready:
            path_response: Path.Response = self.__path_graph_path_service_client.call(request=path_request);
            node_list_size: int = len(path_response.path.node_list);
            self.__log.info(message=f"{PATH_GRAPH_PATH_SERVICE_NAME} Path response : {ros_message_dumps(message=path_response)}, size : {node_list_size}");
            
            return path_response.path;
        else:
            self.__log.error(message=f"{PATH_GRAPH_PATH_SERVICE_NAME} service server is not ready...");
            return;
        
    def navigation_status_subscription_cb(self, status: Status) -> None:
        drive_status: int = status.drive_status;
            
    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        self.__ublox_fix = ublox_fix_cb;

        if self.__ublox_fix is None:
            # error_report: String = String();
            # error_report.data = "451";
            # self.__error_report_publisher.publish(msg=error_report);
            pass;


__all__: list[str] = ["PathProcessor"];