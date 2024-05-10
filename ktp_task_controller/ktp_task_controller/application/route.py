from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.task import Future;
import rclpy.action as rclpy_action;
from rclpy.action.client import ClientGoalHandle;
from rclpy.action.client import ActionClient;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from path_graph_msgs.srv import Path;
from route_msgs.action import RouteToPose;
from action_msgs.msg import GoalStatus;
import route_msgs.msg as route;
from std_msgs.msg import String;
from typing import Any;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.application.status import StatusService;
from ktp_task_controller.utils import ros_message_dumps;
from ktp_task_controller.utils import convert_latlon_to_utm;
from ktp_task_controller.utils import distance_between;
from ktp_task_controller.domain.flags import get_to_source_flag;
from ktp_task_controller.domain.flags import get_to_dest_flag;
from ktp_task_controller.domain.flags import get_returning_flag;
from ktp_task_controller.domain.flags import set_to_source_flag;
from ktp_task_controller.domain.flags import set_to_dest_flag;
from ktp_task_controller.domain.flags import set_returning_flag;
from ktp_task_controller.domain.flags import get_driving_flag;
from ktp_task_controller.domain.flags import set_driving_flag;
from ktp_task_controller.domain.mission import set_mission;
from ktp_task_controller.domain.status import set_driving_status;
from ktp_task_controller.domain.gps import get_gps;


ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME: str = "/rms/ktp/task/goal/cancel";

DRIVE_STATUS_WAIT: int = 0;
DRIVE_STATUS_ON_DRIVE: int = 1;
DRIVE_STATUS_DRIVE_FINISHED: int = 2;
DRIVE_STATUS_CANCELLED: int = 3;
DRIVE_STATUS_OBJECT_DETECTED: int = 4;
DRIVE_STATUS_DRIVE_FAILED: int = 5;
DRIVE_STATUS_MISSION_IMPOSSIBLE: int = 14;


class RouteService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__error_service: ErrorService = ErrorService(node=self.__node);
        self.__status_service: StatusService = StatusService(node=self.__node);
        
        self.__send_goal_future: Future = None;
        self.__result_future: Any = None;
        self.__goal_handle: ClientGoalHandle = None;
        self.__goal_index: int = 0;
        self.__goal_list: list[route.Node] = [];
        self.__goal_list_size: int = 0;
                        
        self.__route_to_pose_action_client: ActionClient = None;
        if self.__route_to_pose_action_client is None:
            route_to_pose_action_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            
            self.__route_to_pose_action_client = rclpy_action.ActionClient(
                node=self.__node,
                action_name=ROUTE_TO_POSE_ACTION_NAME,
                action_type=RouteToPose,
                callback_group=route_to_pose_action_client_cb_group
            );
        else:
            return;
        
        self.__route_to_pose_goal_cancel_subscription: Subscription = None;
        if self.__route_to_pose_goal_cancel_subscription is None:
            route_to_pose_goal_cancel_subsription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            
            self.__route_to_pose_goal_cancel_subscription = self.__node.create_subscription(
                topic=ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME,
                msg_type=String,
                callback_group=route_to_pose_goal_cancel_subsription_cb_group,
                callback=self.route_to_pose_goal_cancel_subscription,
                qos_profile=qos_profile_system_default  
            );
        
    def goal_flush(self) -> None:
        self.__goal_index = 0;
        self.__goal_list.clear();
        self.__goal_list_size = 0;
                    
        set_mission(mission=None);
        set_to_source_flag(flag=False);
        set_to_dest_flag(flag=False);
        set_returning_flag(flag=False);
        set_driving_flag(flag=False);
        set_driving_status(driving_status=DRIVE_STATUS_WAIT);
        self.__log.info("====================== Goal Flushed ======================");
        
    def send_goal(self, path_response: Path.Response) -> None:
        goal: RouteToPose.Goal = RouteToPose.Goal();

        self.__goal_list = path_response.path.node_list;
        self.__goal_list_size = len(self.__goal_list);

        start_node: route.Node = self.__goal_list[self.__goal_index];
        goal.start_node = start_node;
        
        end_node: route.Node = self.__goal_list[self.__goal_index + 1];
        goal.end_node = end_node;
        
        self._send_goal(goal=goal);
        
    def __send_goal(self) -> None:
        goal: RouteToPose.Goal = RouteToPose.Goal();
        
        start_node: route.Node = self.__goal_list[self.__goal_index];
        goal.start_node = start_node;

        end_node: route.Node = self.__goal_list[self.__goal_index + 1];
        goal.end_node = end_node;
        
        self._send_goal(goal=goal);
        
    def _send_goal(self, goal: RouteToPose.Goal) -> None:
        try:            
            is_goal_validate: bool = self.check_goal(start_node_position=goal.start_node.position);
            
            if not is_goal_validate:
                self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} Send Goal goal is invalidate. aborting...");
                self.__error_service.error_report_publish(error_code="450");
                self.__status_service.notify_mission_status_publish(status="Failed");
                self.goal_flush();
                return;

            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Send Goal[{self.__goal_index} / {self.__goal_list_size - 1}]\n{ros_message_dumps(message=goal)}");

            if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
                self.__send_goal_future = self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.__feeback_cb);
                self.__send_goal_future.add_done_callback(callback=self.__goal_response_cb);
            else:
                self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} is not ready...");
                self.__error_service.error_report_publish(error_code="999");
                self.goal_flush();
        except Exception as e:
            self.__log.error(f"Sending Goal {e}");
            self.__error_service.error_report_publish(error_code="999");
            self.goal_flush();
            return;
    
    def check_goal(self, start_node_position: route.Position) -> bool:
        lon: float = start_node_position.longitude;
        lat: float = start_node_position.latitude;
        
        y1, x1 = convert_latlon_to_utm(latitude=get_gps().latitude, longitude=get_gps().longitude);
        y2, x2 = convert_latlon_to_utm(latitude=lat, longitude=lon);
        
        dist: float = distance_between(y1, x1, y2, x2);
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} check goal distance : {dist}");
        
        if dist > 4.0:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} check goal gps is invalidate");
            return False;
        else:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} check goal gps is validate");
            return True;
    
    def __goal_response_cb(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result();
        
        if not goal_handle.accepted:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal rejected");
            self.__error_service.error_report_publish(error_code="201");
            return;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal accepted");

        self.__goal_handle = goal_handle;
        self.__result_future = goal_handle.get_result_async();
        self.__result_future.add_done_callback(callback=self.__result_cb);
        
    def __feeback_cb(self, feedback_msg: RouteToPose.Impl.FeedbackMessage) -> None:
        feedback: RouteToPose.Feedback = feedback_msg.feedback;
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} feedback cb\n{ros_message_dumps(message=feedback)}");

        status_code: int = feedback.status_code;

        if status_code == 1001:
            set_driving_flag(flag=True);
            set_driving_status(driving_status=DRIVE_STATUS_ON_DRIVE);
            
            if get_to_source_flag() is True and get_to_dest_flag() is False and get_returning_flag() is False:
                if self.__goal_index == 0:
                    """
                    대기 장소 -> 상차지(출발지) 주행 출발 시
                    """
                    self.__status_service.notify_mission_status_publish(status="Started");
                    self.__log.info(f"==================================== Started ====================================");
            elif get_to_dest_flag() is True and get_to_source_flag() is False and get_returning_flag() is False:
                if self.__goal_index == 0:
                    """
                    상차지(출발지) -> 하차지 주행 출발 시
                    """
                    self.__status_service.notify_mission_status_publish(status="OnProgress");
                    self.__log.info(f"==================================== OnProgress ====================================");
            elif get_returning_flag() is True and get_to_source_flag() is False and get_to_dest_flag() is False:
                if self.__goal_index == 0:
                    self.__status_service.notify_mission_status_publish(status="End");
                    self.__log.info(f"==================================== End ====================================");
            else:
                return;
        elif status_code == 5001:
            set_driving_flag(flag=False);
            self.__status_service.notify_mission_status_publish(status="Cancelled");
        
    def __result_cb(self, future: Future) -> None:
        result: RouteToPose.Result = future.result().result;
        status: int = future.result().status;

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal succeeded![{self.__goal_index} / {self.__goal_list_size - 1}]\n{ros_message_dumps(message=result)}");
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} Goal failed[{self.__goal_index} / {self.__goal_list_size - 1}]");

        result_code: int = result.result;

        if result_code == 1001:
            if get_to_source_flag() is True and get_to_dest_flag() is False and get_returning_flag() is False:
                """
                대기 장소 -> 상차지(출발지) 주행 도착 시
                """
                is_source_arrived: bool = (self.__goal_index + 1 == self.__goal_list_size - 1);
                if is_source_arrived:
                    set_driving_status(driving_status=DRIVE_STATUS_DRIVE_FINISHED);
                    self.__goal_index = 0;
                    self.__status_service.notify_mission_status_publish(status="SourceArrived");
                    
                    set_to_source_flag(flag=False);
                    set_to_dest_flag(flag=True);
                    set_returning_flag(flag=False);
                    set_driving_flag(flag=False);

                    self.__log.info(f"==================================== Source Arrived ====================================");
                    self.__log.info(f"Flags\n\tto_source : {get_to_source_flag()}\n\tto_dest : {get_to_dest_flag()}\n\treturning : {get_returning_flag()}");
                else:
                    self.__goal_index = self.__goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} To Source will proceed Next Goal [{self.__goal_index} / {self.__goal_list_size - 1}]");
                    self.__send_goal();
            elif get_to_dest_flag() is True and get_to_source_flag() is False and get_returning_flag() is False:
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                is_dest_arrived: bool = (self.__goal_index + 1 == self.__goal_list_size - 1);
                if is_dest_arrived:
                    set_driving_status(driving_status=DRIVE_STATUS_DRIVE_FINISHED);
                    self.__goal_index = 0;
                    self.__status_service.notify_mission_status_publish(status="DestArrived");
                    
                    set_to_source_flag(flag=False);
                    set_to_dest_flag(flag=False);
                    set_returning_flag(flag=True);
                    set_driving_flag(flag=False);

                    self.__log.info(f"==================================== Dest Arrived ====================================");
                    self.__log.info(f"Flags\n\tto_source : {get_to_source_flag()}\n\tto_dest : {get_to_dest_flag()}\n\treturning : {get_returning_flag()}");
                else:
                    self.__goal_index = self.__goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} To Dest will proceed Next Goal [{self.__goal_index} / {self.__goal_list_size - 1}]");
                    self.__send_goal();
            elif get_returning_flag() is True and get_to_source_flag() is False and get_to_dest_flag() is False:
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                is_returning_finished: bool = (self.__goal_index + 1 == self.__goal_list_size - 1);
                if is_returning_finished:
                    set_driving_status(driving_status=DRIVE_STATUS_WAIT);
                    self.goal_flush();

                    self.__log.info(f"==================================== Returning Finished ====================================");
                    self.__log.info(f"Flags\n\tto_source : {get_to_source_flag()}\n\tto_dest : {get_to_dest_flag()}\n\treturning : {get_returning_flag()}");
                else:
                    self.__goal_index = self.__goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Returning will proceed Next Goal [{self.__goal_index} / {self.__goal_list_size - 1}]");
                    self.__send_goal();
            else:
                return;
        elif result_code == 2001 or result_code == 2003 or result_code == 2004 or result_code == 3001:
            self.__status_service.notify_mission_status_publish(status="Failed");
            self.__error_service.error_report_publish(error_code="207");
        elif result_code == 2002:
            self.__status_service.notify_mission_status_publish(status="Failed");
            self.__error_service.error_report_publish(error_code="201");
        else:
            return;
        
    def __cancel_cb(self, future) -> None:
        if get_driving_flag() is True:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} is on Driving");
            return;
        else:
            self.goal_flush();
        cancel_response = future.result();
        if len(cancel_response.goals_canceling) > 0:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal successfully canceled");
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} Goal failed canceling");
        
    def route_to_pose_goal_cancel_subscription(self, goal_cancel_cb: String) -> None:
        try:
            future: Future = self.__goal_handle.cancel_goal_async();
            future.add_done_callback(self.__cancel_cb);
            self.__log.info(f"{ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME} goal cancelled");
        except AttributeError as ate:
            self.__log.error(f"{ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME} : {ate}");
            return;


__all__: list[str] = ["RouteService"];