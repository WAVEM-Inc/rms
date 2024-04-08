import json
from os import path;
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
from std_msgs.msg import Bool;
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
    NAVIGATION_STATUS_READY_TO_MOVE,
    LOOK_UP_PATH_SERVICE_NAME,
    NAVIGATION_STATUS_TOPIC_NAME,
    PATH_GRAPH_PATH_SERVICE_NAME,
    PATH_GRAPH_GRAPH_SERVICE_NAME,
    READY_TO_NAVIGATION_TOPIC_NAME,
    TASK_MANAGER_NODE_NAME,
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

from ktp_task_msgs.srv import LookUpPath;
from ktp_task_msgs.srv import RegisterNavigation;
from ktp_task_msgs.msg import NavigationStatus;


class NavigationProcessor:

    def __init__(self, node: Node, mission_processor: MissionProcessor) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.__mission_processor: MissionProcessor = mission_processor;
        
        device_id_parameter: str = self.__node.get_parameter(name="device_id").get_parameter_value().string_value;
        map_id_parameter: str = self.__node.get_parameter(name="map_id").get_parameter_value().string_value;
        
        path_waiting_place_to_source_parameter: str | Any = self.__node.get_parameter(name="path_waiting_place_to_source").get_parameter_value().string_value;
        self.__path_waiting_place_to_source: str = f"{device_id_parameter}{path_waiting_place_to_source_parameter}";
        
        path_source_to_goal_parmater: str | Any = self.__node.get_parameter(name="path_source_to_goal").get_parameter_value().string_value;
        self.__path_source_to_goal: str = f"{device_id_parameter}{path_source_to_goal_parmater}";
        
        path_goal_to_waiting_place_parameter: str | Any = self.__node.get_parameter(name="path_goal_to_waiting_place").get_parameter_value().string_value;
        self.__path_goal_to_waiting_place: str = f"{device_id_parameter}{path_goal_to_waiting_place_parameter}";
        
        self.__node_list: list[route.Node] = [];
        self.__node_index: int = 0;
        self.__node_list_size: int = 0;
        
        self.__navigation_status: NavigationStatus = None;
        
        self.__path_response: Path.Response = None;
        self.__path_response_list_size: int = 0;
        self.__route_to_pose_send_goal_future: Future = None;
        self.__route_to_pose_action_get_result_future: Any = None;
        self.__route_to_pose_goal_index: int = 0;

        self.__drive_status: int = DRIVE_STATUS_WAIT;
        self.__drive_current: Tuple[str, str] = ("", "");
        
        ready_to_move_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ready_to_move_subscription: Subscription = self.__node.create_subscription(
            topic=READY_TO_NAVIGATION_TOPIC_NAME,
            msg_type=Bool,
            callback_group=ready_to_move_subscription_cb_group,
            callback=self.ready_to_move_subscription_cb,
            qos_profile=qos_profile_system_default
        );
        
        route_to_pose_action_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__route_to_pose_action_client: ActionClient = rclpy_action.ActionClient(
            node=self.__node,
            action_name=ROUTE_TO_POSE_ACTION_NAME,
            action_type=RouteToPose,
            callback_group=route_to_pose_action_client_cb_group
        );
        
        look_up_path_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__look_up_path_service_client: Client = self.__node.create_client(
            srv_name=LOOK_UP_PATH_SERVICE_NAME,
            srv_type=look_up_path_service_client_cb_group,
            callback_group=look_up_path_service_client_cb_group,
            qos_profile=qos_profile_services_default
        );
        
        register_navigation_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__register_navigation_service_client: Client = self.__node.create_client(
            srv_name=f"/{TASK_MANAGER_NODE_NAME}/register/navigation",
            srv_type=RegisterNavigation,
            callback_group=register_navigation_service_client_cb_group,
            qos_profile=qos_profile_services_default
        );
        
        navigation_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__navigation_status_subscription: Subscription = self.__node.create_subscription(
            topic="/rms/ktp/task/navigation/status",
            msg_type=NavigationStatus,
            callback_group=navigation_status_subscription_cb_group,
            callback=self.navigation_status_subscription_cb,
            qos_profile=qos_profile_system_default
        );
        
    def ready_to_move_subscription_cb(self, ready_to_move_cb: Bool) -> None:
        trigger: bool = ready_to_move_cb.data;
        self.__log.info(message=f"{READY_TO_NAVIGATION_TOPIC_NAME} ready_to_move_cb : {trigger}");
        
        if trigger:
            self.__node_list = self.look_up_path_service_request(path_key=self.__path_waiting_place_to_source);
            self.__node_list_size = len(self.__node_list);
            self.register_navigation_service_request( path_key=self.__path_waiting_place_to_source);
            self.route_to_pose_send_goal();
        else:
            return;
    
    def register_navigation_service_request(self, path_key: str) -> None:
        register_navigation_request: RegisterNavigation.Request = RegisterNavigation.Request();
        register_navigation_request.path_key = path_key;
        
        is_register_navigation_service_server_ready: bool = self.__register_navigation_service_client.wait_for_service(timeout_sec=0.75);
        
        if is_register_navigation_service_server_ready:
            navigation_response: RegisterNavigation.Response = self.__register_navigation_service_client.call(request=register_navigation_request);
            
            result: bool = navigation_response.result;
            
            if result:
                self.__log.info(message=f"/{TASK_MANAGER_NODE_NAME}/register/navigation register request suceeded");
            else:
                self.__log.error(message=f"/{TASK_MANAGER_NODE_NAME}/register/navigation register request failed...");
                return;
        else:
            return;
    
    def navigation_status_subscription_cb(self, navigation_status: NavigationStatus) -> None:
        self.__navigation_status = navigation_status;
        pass;

    def route_to_pose_send_goal(self) -> None:
        goal: RouteToPose.Goal = RouteToPose.Goal();
        
        start_node: route.Node = self.__node_list[self.__route_to_pose_goal_index];
        goal.start_node = start_node;

        end_node: route.Node = self.__node_list[self.__route_to_pose_goal_index + 1];
        goal.end_node = end_node;

        self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Send Goal[{self.__route_to_pose_goal_index}]\n{ros_message_dumps(message=goal)}");

        if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
            self.__route_to_pose_send_goal_future = self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.route_to_pose_feedback_cb);
            self.__route_to_pose_send_goal_future.add_done_callback(callback=self.goal_response_callback);
        else:
            self.__log.error(message=f"{ROUTE_TO_POSE_ACTION_NAME} is not ready...");

    def goal_response_callback(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result();
        if not goal_handle.accepted:
            self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Goal rejected");
            return;

        self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Goal accepted");

        self.__route_to_pose_action_get_result_future = goal_handle.get_result_async();
        self.__route_to_pose_action_get_result_future.add_done_callback(callback=self.route_to_pose_get_result_cb);

    def route_to_pose_feedback_cb(self, feedback_msg: RouteToPose.Impl.FeedbackMessage) -> None:
        feedback: RouteToPose.Feedback = feedback_msg.feedback;
        self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} feedback cb\n{ros_message_dumps(message=feedback)}");
        
        status_code: int = feedback.status_code;

        if status_code == 1001:  # 출발
            # 정상 주행 중 : 1
            self.__drive_status = DRIVE_STATUS_ON_DRIVE;
            self.__drive_current = (
                self.__node_list[self.__route_to_pose_goal_index].node_id,
                self.__node_list[self.__route_to_pose_goal_index + 1].node_id);
            self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Feedback Current Drive Status\n\t"
                            f"Drive Status : {self.__drive_status}\n\t"
                            f"Drive Current: {self.__drive_current}");

            if self.__navigation_status.path_source == "waiting" and self.__navigation_status.path_goal == "source":
                if self.__route_to_pose_goal_index == 0:
                    """
                    대기 장소 -> 상차지(출발지) 주행 출발 시
                    """
                    self.__mission_processor.notify_mission_status_publish(status="Started");
                    self.__log.info(message=f"==================================== Started ====================================");
            elif self.__navigation_status.path_source == "source" and self.__navigation_status.path_goal == "goal":
                if self.__route_to_pose_goal_index == 0:
                    """
                    상차지(출발지) -> 하차지 주행 출발 시
                    """
                    self.__mission_processor.notify_mission_status_publish(status="OnProgress");
                    self.__log.info(message=f"==================================== OnProgress ====================================");
            else:
                return;
        elif status_code == 5001:  # 취소
            # 주행 취소 : 3
            self.__drive_status = DRIVE_STATUS_CANCELLED;
            self.__mission_processor.notify_mission_status_publish(status="Cancelled");

    def route_to_pose_get_result_cb(self, future: Future) -> None:
        result: RouteToPose.Result = future.result().result;
        status: int = future.result().status;

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Goal succeeded![{self.__route_to_pose_goal_index}]\n{ros_message_dumps(message=result)}");
        else:
            self.__log.error(message=f"{ROUTE_TO_POSE_ACTION_NAME} Goal failed[{self.__route_to_pose_goal_index}]");

        result_code: int = result.result;

        if result_code == 1001:  # 도착
            self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Result Last Drive Status\n\t"
                            f"Drive Status : {self.__drive_status}\n\t"
                            f"Drive Current: {self.__drive_current}");
            if self.__navigation_status.path_source == "waiting" and self.__navigation_status.path_goal == "source":
                """
                대기 장소 -> 상차지(출발지) 주행 도착 시
                """
                is_source_arrived: bool = (self.__route_to_pose_goal_index + 1 == self.__path_response_list_size - 1);
                if is_source_arrived:
                    self.__node_list = self.look_up_path_service_request(path_key=self.__path_source_to_goal);
                    self.__node_list_size = len(self.__node_list);
                    self.register_navigation_service_request(path_key=self.__path_source_to_goal);
                    
                    self.__drive_status = DRIVE_STATUS_DRIVE_FINISHED;
                    current_node_list: list[route.Node] = self.__path_response.path.node_list;
                    self.__drive_current = (
                        current_node_list[self.__route_to_pose_goal_index].node_id,
                        current_node_list[self.__route_to_pose_goal_index + 1].node_id);

                    self.__route_to_pose_goal_index = 0;
                    self.__mission_processor.notify_mission_status_publish(status="SourceArrived");
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;

                    self.__log.info(message=f"==================================== Source Arrived ====================================");
                    self.__log.info(message=f"Flags\n\tto_source : {to_source_flag}\n\tto_dest : {to_dest_flag}\n\treturning : {returning_flag}");
                    self.__log.info(message=f"Drive\n\tdrive_status : {self.__drive_status}\n\tdrive_current : {self.__drive_current}");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} To Source will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            elif self.__navigation_status.path_source == "source" and self.__navigation_status.path_goal == "goal":
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                is_dest_arrived: bool = (self.__route_to_pose_goal_index + 1 == self.__path_response_list_size - 1);
                if is_dest_arrived:
                    self.__node_list = self.look_up_path_service_request(path_key=self.__path_goal_to_waiting_place);
                    self.__node_list_size = len(self.__node_list);
                    self.register_navigation_service_request(path_key=self.__path_goal_to_waiting_place);
                    
                    self.__drive_status = DRIVE_STATUS_DRIVE_FINISHED;
                    current_node_list: list[route.Node] = self.__path_response.path.node_list;
                    self.__drive_current = (
                        current_node_list[self.__route_to_pose_goal_index].node_id,
                        current_node_list[self.__route_to_pose_goal_index + 1].node_id);

                    self.__route_to_pose_goal_index = 0;
                    self.__mission_processor.notify_mission_status_publish(status="DestArrived");
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;

                    self.__log.info(message=f"==================================== Dest Arrived ====================================");
                    self.__log.info(message=f"Flags\n\tto_source : {to_source_flag}\n\tto_dest : {to_dest_flag}\n\treturning : {returning_flag}");
                    self.__log.info(message=f"Drive\n\tdrive_status : {self.__drive_status}\n\tdrive_current : {self.__drive_current}");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} To Dest will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            elif self.__navigation_status.path_source == "goal" and self.__navigation_status.path_goal == "waiting":
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                is_returning_finished: bool = (self.__route_to_pose_goal_index + 1 == self.__path_response_list_size - 1);
                if is_returning_finished:
                    self.register_navigation_service_request(path_key="");
                    
                    self.__route_to_pose_goal_index = 0;
                    self.__path_response = None;
                    self.__drive_status = DRIVE_STATUS_WAIT;
                    self.__drive_current = ("", "");
                    self.__path_response_list_size = 0;
                    self.__mission = None;
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;

                    self.__log.info(message=f"==================================== Returning Finished ====================================");
                    self.__log.info(message=f"Flags\n\tto_source : {to_source_flag}\n\tto_dest : {to_dest_flag}\n\treturning : {returning_flag}");
                    self.__log.info(message=f"Drive\n\tdrive_status : {self.__drive_status}\n\tdrive_current : {self.__drive_current}");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(message=f"{ROUTE_TO_POSE_ACTION_NAME} Returning will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            else:
                return;
        elif result_code == 2001:  # 실패
            self.__mission_processor.notify_mission_status_publish(status="Failed");
            
    def look_up_path_service_request(self, path_key: str) -> list[route.Node]:
        self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME} request with path_key : {path_key}");
        
        look_up_path_request: LookUpPath.Request = LookUpPath.Request();
        look_up_path_request.path_key = path_key;
        
        is_look_up_path_service_server_ready: bool = self.__look_up_path_service_client.wait_for_service(timeout_sec=0.75);
        
        if is_look_up_path_service_server_ready:
            look_up_path_response: LookUpPath.Response = self.__look_up_path_service_client.call(request=look_up_path_request);
            
            node_list: list[route.Node] = look_up_path_response.path.node_list;
            node_list_size: int = len(self.__node_list);
            self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME} Response node_list_size : {node_list_size}");
            
            for node_index, node in enumerate(node_list):
                self.__log.info(message=f"{LOOK_UP_PATH_SERVICE_NAME} Response node_list[{node_index}]\n{ros_message_dumps(message=node)}");
            
            if not node_list:
                return None;
            else:
                return node_list;
        else:
            return None;
        
        
__all__: list[str] = ["NavigationProcessor"];