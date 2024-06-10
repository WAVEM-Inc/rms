from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.task import Future;
from rclpy.timer import Timer;
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
from sensor_msgs.msg import NavSatFix;
from geometry_msgs.msg import PoseStamped;
from typing import Any;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.application.status import StatusService;
from ktp_task_controller.application.path import PathService;
from ktp_task_controller.utils import convert_latlon_to_utm;
from ktp_task_controller.utils import distance_between;
from ktp_task_controller.domain.flags import get_driving_flag;
from ktp_task_controller.domain.flags import set_driving_flag;
from ktp_task_controller.domain.mission import set_mission;
from ktp_task_controller.domain.status import set_driving_status;
from ktp_task_controller.domain.status import set_mission_total_distance;
from ktp_task_controller.domain.gps import get_gps;
from ktp_task_controller.domain.mission import get_mission;
from ktp_task_controller.domain.status import set_last_arrived_node_id;
from ktp_task_controller.domain.status import get_is_mission_canceled;
from ktp_task_controller.domain.status import set_is_mission_canceled;
from ktp_task_controller.utils import ros_message_dumps;


ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME: str = "/rms/ktp/task/goal/cancel";
ODOM_EULAR_TOPIC_NAME: str = "/drive/odom/eular";

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
        
        self.__param_map_id: str = self.__node.get_parameter(name="map_id").get_parameter_value().string_value;
        self.__param_initial_node: str = self.__node.get_parameter(name="initial_node").get_parameter_value().string_value;
        
        last_arrived_node_id: str = f"NO-{self.__param_map_id}-{self.__param_initial_node}";
        set_last_arrived_node_id(last_arrived_node_id=last_arrived_node_id);
        
        self.__param_goal_validation_limit: float = self.__node.get_parameter(name="goal_validation_limit").get_parameter_value().double_value;
        
        self.__error_service: ErrorService = ErrorService(node=self.__node);
        self.__status_service: StatusService = StatusService(node=self.__node);
        self.__path_service: PathService = PathService(node=self.__node);
        
        self.__current_distance: float = 0.0;
        self.__mission_start_distance: float = 0.0;
        self.__mission_end_distance: float = 0.0;
        
        self.__current_goal: RouteToPose.Goal = RouteToPose.Goal();
        self.__send_goal_future: Future = None;
        self.__result_future: Any = None;
        self.__goal_handle: ClientGoalHandle = None;
        self.__goal_index: int = 0;
        self.__goal_list: list[route.Node] = [];
        self.__goal_list_size: int = 0;
        self.__is_goal_odom_to_gps: bool = False;
        self.__is_goal_need_to_retry: bool = False;
        self.__odom_goal_retry_count: int = 0;
        self.__is_need_to_return: bool = False;
        
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
        
        self.__route_to_pose_odom_goal_retry_timer: Timer = None
        if self.__route_to_pose_odom_goal_retry_timer is None:
            route_to_pose_odom_goal_retry_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            
            self.__route_to_pose_odom_goal_retry_timer = self.__node.create_timer(
                timer_period_sec=1.0,
                callback_group=route_to_pose_odom_goal_retry_timer_cb_group,
                callback=self.route_to_pose_odom_goal_retry_timer_cb
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
        else:
            return;
        
        self.__odom_eular_subscription: Subscription = None;
        if self.__odom_eular_subscription is None:
            odom_eular_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            
            self.__odom_eular_subscription = self.__node.create_subscription(
                topic=ODOM_EULAR_TOPIC_NAME,
                msg_type=PoseStamped,
                callback_group=odom_eular_subscription_cb_group,
                callback=self.odom_eular_subscription_cb,
                qos_profile=qos_profile_system_default
            );
        else:
            return;
        
    def goal_flush(self) -> None:
        self.__goal_index = 0;
        self.__goal_list.clear();
        self.__goal_list_size = 0;
                    
        # set_driving_flag(flag=False);
        # set_driving_status(driving_status=DRIVE_STATUS_WAIT);
    
    def mission_flush(self) -> None:
        set_mission(mission=None);
        self.__log.info("====================== Mission Flushed ======================");
        
    def send_goal(self, path_response: Path.Response) -> None:
        try:
            goal: RouteToPose.Goal = RouteToPose.Goal();

            self.__goal_list = path_response.path.node_list;
            self.__goal_list_size = len(self.__goal_list);

            start_node: route.Node = self.__goal_list[self.__goal_index];
            goal.start_node = start_node;
            
            end_node: route.Node = self.__goal_list[self.__goal_index + 1];
            goal.end_node = end_node;
            
            self._send_goal(goal=goal);
        except IndexError as ide:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} : {ide}");
            self.__error_service.error_report_publish(error_code="450");
            self.__status_service.notify_mission_status_publish(status="Failed");
            self.mission_flush();
            return;
        
    def __send_goal(self) -> None:
        try:
            goal: RouteToPose.Goal = RouteToPose.Goal();
        
            start_node: route.Node = self.__goal_list[self.__goal_index];
            goal.start_node = start_node;

            end_node: route.Node = self.__goal_list[self.__goal_index + 1];
            goal.end_node = end_node;
            
            self._send_goal(goal=goal);
        except IndexError as ide:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} : {ide}");
            self.__error_service.error_report_publish(error_code="450");
            self.__status_service.notify_mission_status_publish(status="Failed");
            self.mission_flush();
            return;
        
    def _send_goal(self, goal: RouteToPose.Goal) -> None:
        try:
            # if get_gps() is not None:
            #     is_goal_validate: bool = self.check_goal(start_node_position=goal.start_node.position);
                
            #     if not is_goal_validate:
            #         self.__is_goal_odom_to_gps = goal.start_node.driving_option == "odom" and goal.end_node.driving_option == "gps";
                    
            #         if self.__is_goal_odom_to_gps:
            #             self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Odom to GPS is InValid. Need To Retry");
            #             self.__is_goal_need_to_retry = True;
            #             return;
            #         else:
            #             self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} Send Goal goal is invalidate. aborting...");
            #             self.__is_goal_need_to_retry = False;
                        # self.__error_service.error_report_publish(error_code="450");
                        # self.__status_service.notify_mission_status_publish(status="Failed");
            #             self.goal_flush();
            #             return;
            # else:
            #     self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Sending Goal GPS is None");
            #     self.__error_service.error_report_publish(error_code="450");
            #     self.__status_service.notify_mission_status_publish(status="Failed");
            #     pass;

            self.__current_goal = goal;
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Send Goal[{self.__goal_index} / {self.__goal_list_size - 1}]"
                            f"\n\t ============= {goal.start_node.node_id} -> {goal.end_node.node_id} =============");

            if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
                self.__send_goal_future = self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.__feedback_cb);
                self.__send_goal_future.add_done_callback(callback=self.__goal_response_cb);
            else:
                self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} is not ready...");
                self.__error_service.error_report_publish(error_code="999");
                self.goal_flush();
        except Exception as e:
            self.__log.error(f"Sending Goal : {e}");
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
        
        if dist > self.__param_goal_validation_limit:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} check goal gps is invalidate");
            return False;
        else:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} check goal gps is validate");
            return True;
        
    def start_mission(self) -> None:
        if self.__is_goal_need_to_retry == False:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} started capture mission_distance");
            self.__mission_start_distance = self.__current_distance;
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} mission_start_distance : {self.__mission_start_distance}");
                            
            self.__status_service.notify_mission_status_publish(status="Started");
            self.__log.info(f"==================================== Mission Started ====================================");
        else:
            return;
    
    def end_mission(self) -> None:
        self.calculate_mission_total_distance();
        self.__status_service.notify_mission_status_publish(status="End");
        self.__log.info(f"==================================== Mission End ====================================");
        self.__mission_start_distance = 0.0;
        self.__mission_end_distance = 0.0;
        set_mission_total_distance(mission_total_distance=0);
        
    def process_return(self) -> None:
        set_driving_flag(flag=False);
        set_driving_status(driving_status=DRIVE_STATUS_WAIT);
                        
        self.__goal_index = 0;
        self.__log.info(f"==================================== Returning Finished ====================================");
        self.end_mission();
        self.goal_flush();
        self.mission_flush();
    
    def process_no_return(self) -> None:
        set_driving_flag(flag=False);
        set_driving_status(driving_status=DRIVE_STATUS_WAIT);
                        
        self.__goal_index = 0;
        self.__log.info(f"==================================== No Return ====================================");
        self.end_mission();
        self.goal_flush();
        self.mission_flush();
    
    def calculate_mission_total_distance(self) -> None:
        try:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} started capture mission_distance");
            self.__mission_end_distance = self.__current_distance;
            mission_total_distance: int = int(abs(self.__mission_end_distance - self.__mission_start_distance));
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} mission_total_distance : {mission_total_distance}");
            set_mission_total_distance(mission_total_distance=mission_total_distance);
        except AssertionError as ate:
            self.__log.error(f"{ate}");
            return;
    
    def __goal_response_cb(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result();
        
        if not goal_handle.accepted:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal rejected");
            self.__error_service.error_report_publish(error_code="201");
            return;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal accepted\n");

        self.__goal_handle = goal_handle;
        self.__result_future = goal_handle.get_result_async();
        self.__result_future.add_done_callback(callback=self.__result_cb);
        
    def __feedback_cb(self, feedback_msg: RouteToPose.Impl.FeedbackMessage) -> None:
        feedback: RouteToPose.Feedback = feedback_msg.feedback;
        status_code: int = feedback.status_code;
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} feedback status_code {status_code}\n");

        if status_code == 1001:
            set_driving_flag(flag=True);
            
            if not get_is_mission_canceled():
                set_driving_status(driving_status=DRIVE_STATUS_ON_DRIVE);
                
            is_mission_returning_task: bool = get_mission().task[0].task_code == "returning";
            
            if self.__current_goal.start_node.node_id == get_mission().task[0].task_data.source and not is_mission_returning_task:
                if self.__goal_index == 0:
                    """
                    Source -> Goal 주행 출발 시
                    """
                    if self.__is_goal_need_to_retry == False:
                        self.__status_service.notify_mission_status_publish(status="OnProgress");
                        self.__log.info(f"==================================== OnProgress ====================================");
                    else:
                        return;
            elif get_is_mission_canceled():
                set_driving_flag(flag=True);
                set_driving_status(driving_status=DRIVE_STATUS_CANCELLED);
            else:
                return;
        elif status_code == 5001:
            set_driving_flag(flag=False);
            set_driving_status(driving_status=DRIVE_STATUS_CANCELLED);
        
    def __result_cb(self, future: Future) -> None:
        result: RouteToPose.Result = future.result().result;
        status: int = future.result().status;

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal succeeded![{self.__goal_index} / {self.__goal_list_size - 1}]\n");
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} Goal failed[{self.__goal_index} / {self.__goal_list_size - 1}]\n");

        result_code: int = result.result;

        if result_code == 1001:
            set_last_arrived_node_id(last_arrived_node_id=self.__current_goal.end_node.node_id);
            
            is_mission_returning_task: bool = get_mission().task[0].task_code == "returning";
            is_end_node_is_mission_source: bool = self.__current_goal.end_node.node_id == get_mission().task[0].task_data.source;
            is_end_node_is_mission_goal: bool = self.__current_goal.end_node.node_id == get_mission().task[0].task_data.goal[0];
            is_end_node_is_waiting_area: bool = not get_is_mission_canceled() and self.__current_goal.end_node.node_id == f"NO-{self.__param_map_id}-{self.__param_initial_node}";
            is_return_node_via_mission_cancel: bool = False;
            
            if len(self.__current_goal.end_node.detection_range) != 0:
                if get_is_mission_canceled() is True and self.__current_goal.end_node.detection_range[0].action_code == "stop":
                    is_return_node_via_mission_cancel = True;
                    pass;
                else:
                    is_return_node_via_mission_cancel = False;
                    pass;
            else:
                is_return_node_via_mission_cancel = False;
                pass;
            
            if is_end_node_is_mission_source and not get_is_mission_canceled():
                """
                목적지가 Mission Source 일 경우
                """
                if not is_mission_returning_task:
                    """
                    대기 장소 -> Source 주행 도착 시
                    """
                    set_driving_flag(flag=False);
                    set_driving_status(driving_status=DRIVE_STATUS_DRIVE_FINISHED);
                    self.__goal_index = 0;
                        
                    self.__status_service.notify_mission_status_publish(status="SourceArrived");
                    self.__log.info(f"==================================== Source Arrived ====================================");
                    self.goal_flush();
                    return;
                else:
                    pass;
            elif is_end_node_is_mission_goal and not get_is_mission_canceled():
                """
                목적지가 Mission Source 일 경우
                """
                if not is_mission_returning_task:
                    """
                    Source -> Goal 주행 도착 시
                    """
                    set_driving_flag(flag=False);
                    set_driving_status(driving_status=DRIVE_STATUS_DRIVE_FINISHED);
                    self.__goal_index = 0;
                        
                    self.__status_service.notify_mission_status_publish(status="DestArrived");
                    self.__log.info(f"==================================== Dest Arrived ====================================");
                    self.goal_flush();
                    return;
                else:
                    if not get_is_mission_canceled() and self.__current_goal.end_node.node_id == f"NO-{self.__param_map_id}-{self.__param_initial_node}":
                        self.process_return();
                        return;
                    else:
                        pass;
            elif is_end_node_is_waiting_area and not get_is_mission_canceled():
                """
                목적지가 대기 장소 일 경우
                """
                self.process_return();
                return;
            elif is_return_node_via_mission_cancel:
                """
                목적지가 임무 취소 후 회차 노드 일 경우
                """
                set_driving_flag(flag=False);
                
                path_request: Path.Request = Path.Request();
                path_request.start_node = self.__current_goal.end_node.node_id;
                path_request.end_node = self.__goal_list[0].node_id;

                path_response: Path.Response = self.__path_service.convert_path_request(path_request=path_request);
                
                if path_response != None or len(path_response.path.node_list) != 0:
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Return Via Mission Canceled Path Response\n{ros_message_dumps(message=path_response)}");

                    if get_driving_flag() != True:
                        self.goal_flush();
                        self.send_goal(path_response=path_response);
                    else:
                        self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} is already driving");
                        return;
                else:
                    self.__status_service.notify_mission_status_publish(status="Failed");
                    self.__error_service.error_report_publish(error_code="201");
                    return;
            elif get_is_mission_canceled() and self.__current_goal.end_node.node_id == self.__goal_list[self.__goal_list_size - 1].node_id:
                set_driving_flag(flag=False);
                set_driving_status(driving_status=DRIVE_STATUS_WAIT);
                set_is_mission_canceled(is_mission_canceled=False);
                self.goal_flush();
                self.mission_flush();
                self.__log.info(f"=============== Cancel Return Finished ===============");
            else:
                self.__goal_index = self.__goal_index + 1;
                self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} RTP will proceed Next Goal [{self.__goal_index} / {self.__goal_list_size - 1}]");
                self.__send_goal();
                return;
        elif result_code == 2001 or result_code == 2003 or result_code == 2004 or result_code == 3001 and not get_is_mission_canceled():
            self.__status_service.notify_mission_status_publish(status="Failed");
            self.__error_service.error_report_publish(error_code="207");
        elif result_code == 2002 and not get_is_mission_canceled():
            self.__status_service.notify_mission_status_publish(status="Failed");
            self.__error_service.error_report_publish(error_code="201");
        else:
            return;
    
    def route_to_pose_odom_goal_retry_timer_cb(self) -> None:
        if self.__is_goal_need_to_retry:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Retrying Sending Odom to GPS Goal[{self.__odom_goal_retry_count}]");
            self.__log.info("------------------------------------------------------------------------\n");
            self.__send_goal();
            self.__odom_goal_retry_count = self.__odom_goal_retry_count + 1;
            
            if self.__odom_goal_retry_count == 10:
                self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Odom to GPS Goal Failed");
                self.__is_goal_need_to_retry = False;
                self.__error_service.error_report_publish(error_code="450");
                self.__status_service.notify_mission_status_publish(status="Failed");
                self.goal_flush();
                self.__odom_goal_retry_count = 0;
            else:
                return;
        else:
            self.__odom_goal_retry_count = 0;
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
            self.cancel_goal();
            self.__log.info(f"{ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME} goal cancelled");
        except AttributeError as ate:
            self.__log.error(f"{ROUTE_TO_POSE_GOAL_CANCEL_TOPIC_NAME} : {ate}");
            return;

    def odom_eular_subscription_cb(self, odom_eular_cb: PoseStamped) -> None:
        self.__current_distance = odom_eular_cb.pose.position.y;
    
    def cancel_goal(self) -> None:
        try:
            future: Future = self.__goal_handle.cancel_goal_async();
            future.add_done_callback(self.__cancel_cb);
        except Exception as e:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} : {e}");
            return;
    
    def cancel_mission(self) -> None:
        try:
            self.cancel_goal();
            self.__status_service.notify_mission_status_publish(status="Cancelled");
            self.goal_flush();
            self.__goal_handle = None;
            self.mission_flush();
        except Exception as e:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} : {e}");
            return;
    

__all__: list[str] = ["RouteService"];
