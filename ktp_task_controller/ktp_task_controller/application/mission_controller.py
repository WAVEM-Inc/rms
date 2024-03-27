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
from ktp_data_msgs.srv import AssignMission;
from path_graph_msgs.srv import Path;
from sensor_msgs.msg import NavSatFix;
from route_msgs.action import RouteToPose;
import route_msgs.msg as route;
from typing import Any;
from ktp_task_controller.application.utils import ros_message_dumps;
from ktp_task_controller.application.utils import get_current_time;

NODE_NAME: str = "ktp_task_controller";
ASSIGN_MISSION_SERVICE_NAME: str = f"/{NODE_NAME}/assign/mission";
PATH_GRAPH_PATH_SERVICE_NAME: str = "/path_graph_msgs/path";
UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
NOTIFY_MISSION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/mission/status";
NOTIFY_NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/navigation/status";

to_source_flag: bool = False;
to_dest_flag: bool = False;
return_flag: bool = False;


class MissionController:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mission: Mission = Mission();
        self.__ublox_fix: NavSatFix = None;
        self.__path_response: Path.Response = None;
        self.__path_response_list_size: int = 0;
        self.__route_to_pose_action_send_goal_future: Any = None;
        self.__route_to_pose_action_get_result_future: Any = None;
        self.__route_to_pose_to_source_index: int = 0;

        assign_mission_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            callback_group=assign_mission_service_cb_group,
            callback=self.assign_mission_service_cb
        );

        route_to_pose_action_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__route_to_pose_action_client: ActionClient = rclpy_action.ActionClient(
            node=self.__node,
            action_name=ROUTE_TO_POSE_ACTION_NAME,
            action_type=RouteToPose,
            callback_group=route_to_pose_action_client_cb_group
        );

        path_graph_path_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__path_graph_path_service_client: Client = self.__node.create_client(
            srv_name=PATH_GRAPH_PATH_SERVICE_NAME,
            srv_type=Path,
            callback_group=path_graph_path_service_client_cb_group,
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

        notify_mission_status_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_mission_status_publisher: Publisher = self.__node.create_publisher(
            topic=NOTIFY_MISSION_STATUS_TOPIC_NAME,
            msg_type=ServiceStatus,
            callback_group=notify_mission_status_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );
        
        notify_navigation_status_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_navigation_status_publisher: Publisher = self.__node.create_publisher(
            topic=NOTIFY_NAVIGATION_STATUS_TOPIC_NAME,
            msg_type=Status,
            callback_group=notify_navigation_status_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );

        notify_status_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_status_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.8,
            callback_group=notify_status_timer_cb_group,
            callback=self.notify_mission_timer_cb
        );

    def assign_mission_service_cb(self, request: AssignMission.Request, response: AssignMission.Response) -> AssignMission.Response:
        request_mission_json: str = ros_message_dumps(message=request.mission);
        self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} request\n{request_mission_json}");

        self.__mission = message_conversion.populate_instance(json.loads(request_mission_json), Mission());
        
        global to_source_flag;
        to_source_flag = True;

        if self.__ublox_fix is None:
            self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} {UBLOX_FIX_TOPIC_NAME} is None...");
            response.result = False;
        else:
            self.path_graph_path_request();
            response.result = True;

        return response;

    def path_graph_path_request(self) -> None:
        mission_task: MissionTask = self.__mission.task[0];
        self.__log.info(
            f"{PATH_GRAPH_PATH_SERVICE_NAME} Request Task\n{ros_message_dumps(message=mission_task)}");

        global to_source_flag;
        global to_dest_flag;
        global return_flag;

        self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} Current Flags\n\tto_source_flag : {to_source_flag}\n\tto_dest_flag : {to_dest_flag}\n\treturn_flag : {return_flag}\n\tublox : {self.__ublox_fix is None}");

        path_graph_path_request: Path.Request = Path.Request();

        if self.__ublox_fix is not None:
            if to_source_flag is True and to_dest_flag is False and return_flag is False:
                """
                대기 장소 -> 상차지(출발지)
                - position : 현재 GPS
                - start_node : null
                - end_node : task.task_data.source
                """
                path_graph_path_request.position.longitude = 0.0;
                path_graph_path_request.position.latitude = 0.0;

                path_graph_path_request.start_node = "";
                path_graph_path_request.end_node = mission_task.task_data.source;

                self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path Request is ready : {ros_message_dumps(message=path_graph_path_request)}");

                if self.__path_graph_path_service_client.wait_for_service(timeout_sec=0.5):
                    to_source_request_future: Future = self.__path_graph_path_service_client.call_async(request=path_graph_path_request);
                    to_source_request_future_result: Any = to_source_request_future.result();

                    if to_source_request_future_result is not None:
                        to_source_request_response: Path.Response = to_source_request_future_result.result;
                        self.__path_response = to_source_request_response;
                        self.__path_response_list_size = len(self.__path_response.path.node_list);
                        
                        self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path response : {ros_message_dumps(message=to_source_request_response)}, size : {self.__path_response_list_size}");
                        
                        # 대기 장소 -> 상차지(출발지) 첫 번째 Goal 송신
                        self.to_source_send_goal(start_index=self.__route_to_pose_to_source_index, end_index=self.__route_to_pose_to_source_index + 1);
                    else:
                        self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path response is None...");
                        return;
                else:
                    self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} is not ready...");
                    return;

                self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path Request Result : {to_source_request_future_result}");
            elif to_dest_flag is True and to_source_flag is False and return_flag is False:
                """
                상차지(출발지) -> 하차지
                - position : 현재 GPS
                - start_node : task.task_data.source
                - end_node : task.task_data.goal
                """
                path_graph_path_request.position.longitude = self.__ublox_fix.longitude;
                path_graph_path_request.position.latitude = self.__ublox_fix.latitude;

                path_graph_path_request.start_node = mission_task.task_data.source;
                path_graph_path_request.end_node = mission_task.task_data.goal[0];

                self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Goal Path Request is ready : {ros_message_dumps(message=path_graph_path_request)}");

                if self.__path_graph_path_service_client.wait_for_service(timeout_sec=0.5):
                    to_goal_request_future: Future = self.__path_graph_path_service_client.call_async(request=path_graph_path_request);
                    to_goal_request_future_result: Any = to_goal_request_future.result();

                    to_goal_request_response: Path.Response = to_goal_request_future_result.result;
                    self.__path_response = to_goal_request_response;
                    self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Goal Path response : {ros_message_dumps(message=to_goal_request_response)}");
                else:
                    self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} is not ready...");
                    return;

                self.__log.info(
                    f"{PATH_GRAPH_PATH_SERVICE_NAME} To Goal Path Request Result : {to_goal_request_future_result}");
            elif return_flag is True and to_source_flag is False and to_dest_flag is False:
                """
                하차지 -> 대기 장소
                - position : 현재 GPS
                - start_node : task.task_data.goal
                - end_node : null
                """
                path_graph_path_request.position.longitude = self.__ublox_fix.longitude;
                path_graph_path_request.position.latitude = self.__ublox_fix.latitude;

                path_graph_path_request.start_node = mission_task.task_data.goal[0];
                path_graph_path_request.end_node = "";

                self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} Return Path Request is ready : {ros_message_dumps(message=path_graph_path_request)}");

                if self.__path_graph_path_service_client.wait_for_service(timeout_sec=0.5):
                    return_request_future: Future = self.__path_graph_path_service_client.call_async(request=path_graph_path_request);
                    return_request_future_result: Any = return_request_future.result();

                    return_request_response: Path.Response = return_request_future_result.result;
                    self.__path_response = return_request_response;
                    self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} Return Path response : {ros_message_dumps(message=return_request_response)}");
                else:
                    self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} is not ready...");
                    return;

                self.__log.info(
                    f"{PATH_GRAPH_PATH_SERVICE_NAME} Return Path Request Result : {return_request_future_result}");
            else:
                self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} Unknown Flag");
                return;
        else:
            self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} GPS Is None...");
            return;

    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        self.__ublox_fix = ublox_fix_cb;

    def route_to_pose_send_goal(self, goal: RouteToPose.Goal) -> None:
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} send goal : {ros_message_dumps(message=goal)}");
        
        if self.__route_to_pose_action_client.wait_for_server(timeout_sec=0.75):
            self.__route_to_pose_action_client.send_goal_async(goal=goal, feedback_callback=self.route_to_pose_feedback_cb);
        else:
            self.__log.error(f"{ROUTE_TO_POSE_ACTION_NAME} is not ready...");
            
    def to_source_send_goal(self, start_index: int, end_index: int) -> None:
        goal: RouteToPose.Goal = RouteToPose.Goal();
        
        node_list: list[route.Node] = self.__path_response.path.node_list;
        goal.start_node(node_list[start_index]);
        goal.start_node(node_list[end_index]);
        
        self.route_to_pose_send_goal(goal=goal);

    def to_dest_send_goal(self) -> None:
        pass;
    
    def return_send_goal(self) -> None:
        pass;

    def goal_response_callback(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result();
        if not goal_handle.accepted:
            self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal rejected");
            return;

        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Goal accepted");

        self.__route_to_pose_action_get_result_future = goal_handle.get_result_async();
        self.__route_to_pose_action_get_result_future.add_done_callback(callback=self.route_to_pose_get_result_cb);

    def route_to_pose_get_result_cb(self, future: Future) -> None:
        result: RouteToPose.Result = future.result().result;
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} result cb : {ros_message_dumps(message=result)}");
        
        global to_source_flag;
        global to_dest_flag;
        global return_flag;
        
        result_code: int = result.result;
        
        if result_code == 1001: # 도착
            if to_source_flag is True and to_dest_flag is False and return_flag is False:
                """
                대기 장소 -> 상차지(출발지) 주행 도착 시
                """
                self.notify_mission_status_publish(status="SourceArrived");
                self.to_source_send_goal(start_index=self.__route_to_pose_to_source_index, end_index=self.__route_to_pose_to_source_index + 1);
            elif to_dest_flag is True and to_source_flag is False and return_flag is False:
                """
                상차지(출발지) -> 하차지 주행 도착 시
                """
                self.notify_mission_status_publish(status="DestArrived");
            else:
                return;
        elif result_code == 2001: # 실패
            self.notify_mission_status_publish(status="Failed");
        

    def route_to_pose_feedback_cb(self, feedback_msg: RouteToPose.Impl.FeedbackMessage) -> None:
        feedback: RouteToPose.Feedback = feedback_msg.feedback;
        self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} feedback cb : {ros_message_dumps(message=feedback)}");
        
        global to_source_flag;
        global to_dest_flag;
        global return_flag;
        
        status_code: int = feedback.status_code;
        
        if status_code == 1001: # 출발
            if to_source_flag is True and to_dest_flag is False and return_flag is False:
                """
                대기 장소 -> 상차지(출발지) 주행 출발 시
                """
                self.notify_mission_status_publish(status="Started");
            elif to_dest_flag is True and to_source_flag is False and return_flag is False:
                """
                상차지(출발지) -> 하차지 주행 출발 시
                """
                self.notify_mission_status_publish(status="OnProgress");
            else:
                return;
        elif status_code == 5001: # 취소
            self.notify_mission_status_publish(status="Cancelled");
            

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
        
        service_status.task = [service_status_task];
        service_status_task.task_data = service_status_task_data;
        
        self.__log.info(f"{NOTIFY_MISSION_STATUS_TOPIC_NAME} Service Status : {ros_message_dumps(message=service_status)}");
        self.__notify_mission_status_publisher.publish(msg=service_status);
        
    
    def notify_mission_timer_cb(self) -> None:
        pass;
