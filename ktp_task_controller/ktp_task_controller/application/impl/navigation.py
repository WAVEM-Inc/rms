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


class NavigationProcessor:

    def __init__(self, node: Node, mission_processor: MissionProcessor) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mission_processor: MissionProcessor = mission_processor;
        self.__path_response: Path.Response = None;
        self.__path_response_list_size: int = 0;
        self.__route_to_pose_send_goal_future: Future = None;
        self.__route_to_pose_action_get_result_future: Any = None;
        self.__route_to_pose_goal_index: int = 0;

        self.__drive_status: int = DRIVE_STATUS_WAIT;
        self.__drive_current: Tuple[str, str] = ("", "");

        route_to_pose_action_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__route_to_pose_action_client: ActionClient = rclpy_action.ActionClient(
            node=self.__node,
            action_name=ROUTE_TO_POSE_ACTION_NAME,
            action_type=RouteToPose,
            callback_group=route_to_pose_action_client_cb_group
        );

    def route_to_pose_send_goal(self) -> None:
        goal: RouteToPose.Goal = RouteToPose.Goal();

        node_list: list[route.Node] = self.__path_response.path.node_list;

        start_node: route.Node = node_list[self.__route_to_pose_goal_index];
        goal.start_node = start_node;

        end_node: route.Node = node_list[self.__route_to_pose_goal_index + 1];
        goal.end_node = end_node;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Send Goal[{self.__route_to_pose_goal_index}]\n{ros_message_dumps(message=goal)}");

        if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
            self.__route_to_pose_send_goal_future = self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.route_to_pose_feedback_cb);
            self.__route_to_pose_send_goal_future.add_done_callback(callback=self.goal_response_callback);
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} is not ready...");

    def goal_response_callback(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result();
        if not goal_handle.accepted:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal rejected");
            return;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal accepted");

        self.__route_to_pose_action_get_result_future = goal_handle.get_result_async();
        self.__route_to_pose_action_get_result_future.add_done_callback(callback=self.route_to_pose_get_result_cb);

    def route_to_pose_feedback_cb(self, feedback_msg: RouteToPose.Impl.FeedbackMessage) -> None:
        feedback: RouteToPose.Feedback = feedback_msg.feedback;
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} feedback cb\n{ros_message_dumps(message=feedback)}");

        global to_source_flag;
        global to_dest_flag;
        global returning_flag;

        status_code: int = feedback.status_code;

        if status_code == 1001:  # 출발
            # 정상 주행 중 : 1
            self.__drive_status = DRIVE_STATUS_ON_DRIVE;
            current_node_list: list[route.Node] = self.__path_response.path.node_list;
            self.__drive_current = (
                current_node_list[self.__route_to_pose_goal_index].node_id,
                current_node_list[self.__route_to_pose_goal_index + 1].node_id);
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Feedback Current Drive Status\n\t"
                            f"Drive Status : {self.__drive_status}\n\t"
                            f"Drive Current: {self.__drive_current}");

            if to_source_flag is True and to_dest_flag is False and returning_flag is False:
                if self.__route_to_pose_goal_index == 0:
                    """
                    대기 장소 -> 상차지(출발지) 주행 출발 시
                    """
                    self.__mission_processor.notify_mission_status_publish(status="Started");
                    self.__log.info(f"==================================== Started ====================================");
            elif to_dest_flag is True and to_source_flag is False and returning_flag is False:
                if self.__route_to_pose_goal_index == 0:
                    """
                    상차지(출발지) -> 하차지 주행 출발 시
                    """
                    self.__mission_processor.notify_mission_status_publish(status="OnProgress");
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
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal succeeded![{self.__route_to_pose_goal_index}]\n{ros_message_dumps(message=result)}");
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} Goal failed[{self.__route_to_pose_goal_index}]");

        global to_source_flag;
        global to_dest_flag;
        global returning_flag;

        result_code: int = result.result;

        if result_code == 1001:  # 도착
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Result Last Drive Status\n\t"
                            f"Drive Status : {self.__drive_status}\n\t"
                            f"Drive Current: {self.__drive_current}");
            if to_source_flag is True and to_dest_flag is False and returning_flag is False:
                """
                대기 장소 -> 상차지(출발지) 주행 도착 시
                """
                is_source_arrived: bool = (self.__route_to_pose_goal_index + 1 == self.__path_response_list_size - 1);
                if is_source_arrived:
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

                    self.__log.info(f"==================================== Source Arrived ====================================");
                    self.__log.info(f"Flags\n\tto_source : {to_source_flag}\n\tto_dest : {to_dest_flag}\n\treturning : {returning_flag}");
                    self.__log.info(f"Drive\n\tdrive_status : {self.__drive_status}\n\tdrive_current : {self.__drive_current}");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} To Source will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            elif to_dest_flag is True and to_source_flag is False and returning_flag is False:
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                is_dest_arrived: bool = (self.__route_to_pose_goal_index + 1 == self.__path_response_list_size - 1);
                if is_dest_arrived:
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

                    self.__log.info(f"==================================== Dest Arrived ====================================");
                    self.__log.info(f"Flags\n\tto_source : {to_source_flag}\n\tto_dest : {to_dest_flag}\n\treturning : {returning_flag}");
                    self.__log.info(f"Drive\n\tdrive_status : {self.__drive_status}\n\tdrive_current : {self.__drive_current}");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} To Dest will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            elif returning_flag is True and to_source_flag is False and to_dest_flag is False:
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                is_returning_finished: bool = (self.__route_to_pose_goal_index + 1 == self.__path_response_list_size - 1);
                if is_returning_finished:
                    self.__route_to_pose_goal_index = 0;
                    self.__path_response = None;
                    self.__drive_status = DRIVE_STATUS_WAIT;
                    self.__drive_current = ("", "");
                    self.__path_response_list_size = 0;
                    self.__mission = None;
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;

                    self.__log.info(f"==================================== Returning Finished ====================================");
                    self.__log.info(f"Flags\n\tto_source : {to_source_flag}\n\tto_dest : {to_dest_flag}\n\treturning : {returning_flag}");
                    self.__log.info(f"Drive\n\tdrive_status : {self.__drive_status}\n\tdrive_current : {self.__drive_current}");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Returning will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            else:
                return;
        elif result_code == 2001:  # 실패
            self.__mission_processor.notify_mission_status_publish(status="Failed");