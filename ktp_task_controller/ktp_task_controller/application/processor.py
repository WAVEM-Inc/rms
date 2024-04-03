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
from ktp_data_msgs.msg import ControlReportData;
from ktp_data_msgs.msg import ControlReportDataGraphList;
from std_msgs.msg import String;
from path_graph_msgs.srv import Path;
from path_graph_msgs.srv import Graph;
from sensor_msgs.msg import NavSatFix;
from route_msgs.action import RouteToPose;
from action_msgs.msg import GoalStatus;
import route_msgs.msg as route;
from typing import Any;
from typing import Tuple;
from ktp_task_controller.utils import ros_message_dumps;
from ktp_task_controller.utils import get_current_time;

NODE_NAME: str = "ktp_task_controller";
ASSIGN_MISSION_TOPIC_NAME: str = "/rms/ktp/data/assign/mission";
ASSIGN_MISSION_SERVICE_NAME: str = f"/{NODE_NAME}/assign/mission";
PATH_GRAPH_PATH_SERVICE_NAME: str = "/path_graph_msgs/path";
UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
NOTIFY_MISSION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/mission/status";
NOTIFY_NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/navigation/status";
ERROR_REPORT_TOPIC_NAME: str = "/rms/ktp/data/notify/error/status";
ASSIGN_CONTROL_SERVICE_NAME: str = f"/{NODE_NAME}/assign/control";
NOTIFY_CONTROL_REPORT_TOPIC_NAME: str = "/rms/ktp/task/notify/control/report";
PATH_GRAPH_GRAPH_SERVICE_NAME: str = "/path_graph_msgs/graph";
GRAPH_LIST_TOPIC: str = "/rms/ktp/task/notify/graph_list";

CONTROL_CODE_STOP: str = "stop";
CONTROL_CODE_RELEASE: str = "release";
CONTROL_MS_CANCEL: str = "mscancel";
CONTROL_CODE_MOVE_TO_DEST: str = "movetodest";
CONTROL_CODE_MS_COMPLETE: str = "mscomplete";
CONTORL_CODE_GRAPH_SYNC: str = "graphsync";

to_source_flag: bool = False;
to_dest_flag: bool = False;
returning_flag: bool = False;

@staticmethod
def get_to_source_flag() -> bool:
    return to_source_flag;

@staticmethod
def set_to_source_flag(flag: bool) -> None:
    global to_source_flag;
    to_source_flag = flag;

@staticmethod
def get_to_dest_flag() -> bool:
    return to_dest_flag;

@staticmethod
def set_to_dest_flag(flag: bool) -> None:
    global to_dest_flag;
    to_dest_flag = flag;

@staticmethod
def set_returning_flag(flag: bool) -> None:
    global returning_flag;
    returning_flag = flag;

DRIVE_STATUS_WAIT: int = 0;
DRIVE_STATUS_ON_DRIVE: int = 1;
DRIVE_STATUS_DRIVE_FINISHED: int = 2;
DRIVE_STATUS_CANCELLED: int = 3;
DRIVE_STATUS_OBJECT_DETECTED: int = 4;
DRIVE_STATUS_DRIVE_FAILED: int = 5;
DRIVE_STATUS_MISSION_IMPOSSIBLE: int = 14;

KTP_DEVICE_ID: str = "KECDSEMITB001";


class Processor:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mission: Mission = None;
        self.__ublox_fix: NavSatFix = None;
        self.__path_response: Path.Response = None;
        self.__path_response_list_size: int = 0;
        self.__route_to_pose_send_goal_future: Future = None;
        self.__route_to_pose_action_get_result_future: Any = None;
        self.__route_to_pose_goal_index: int = 0;
        self.__drive_status: int = DRIVE_STATUS_WAIT;
        self.__drive_current: Tuple[str, str] = ("", "");

        assign_mission_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            callback_group=assign_mission_service_cb_group,
            callback=self.assign_mission_service_cb,
            qos_profile=qos_profile_services_default
        );

        assign_control_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_service: Service = self.__node.create_service(
            srv_name=ASSIGN_CONTROL_SERVICE_NAME,
            srv_type=AssignControl,
            callback_group=assign_control_service_cb_group,
            callback=self.assign_control_service_cb,
            qos_profile=qos_profile_services_default
        );

        control_report_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__control_report_publisher: Publisher = self.__node.create_publisher(
            topic=NOTIFY_CONTROL_REPORT_TOPIC_NAME,
            msg_type=ControlReport,
            callback_group=control_report_publisher_cb_group,
            qos_profile=qos_profile_system_default
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

        error_report_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__error_report_publisher: Publisher = self.__node.create_publisher(
            topic=ERROR_REPORT_TOPIC_NAME,
            msg_type=String,
            callback_group=error_report_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );

        path_graph_graph_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__path_graph_graph_service_client: Client = self.__node.create_client(
            srv_name=PATH_GRAPH_GRAPH_SERVICE_NAME,
            srv_type=Graph,
            callback_group=path_graph_graph_service_client_cb_group,
            qos_profile=qos_profile_services_default
        );

        graph_list_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_publisher: Publisher = self.__node.create_publisher(
            topic=GRAPH_LIST_TOPIC,
            msg_type=GraphList,
            callback_group=graph_list_publisher_cb_group,
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
                self.error_report_publish(error_code="451");
            else:
                self.command_navigation_with_path();
                response.result = True;

        return response;

    def assign_control_service_cb(self, request: AssignControl.Request, response: AssignControl.Response) -> AssignControl.Response:
        request_control_json: str = ros_message_dumps(message=request.control);
        self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} request\n{request_control_json}");

        control: Control = message_conversion.populate_instance(json.loads(request_control_json), Control());
        control_code: str = control.control_code;
        self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} cb control_code : {control_code}");

        if control_code == CONTROL_CODE_STOP:
            pass;
        elif control_code == CONTROL_CODE_RELEASE:
            pass;
        elif control_code == CONTROL_MS_CANCEL:
            pass;
        elif control_code == CONTROL_CODE_MOVE_TO_DEST:
            if get_to_source_flag() is True or get_to_dest_flag() is True:
                self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Robot is Moving To Source");
                self.control_report_publish(control=control, control_type="control", response_code=400);
                response.result = False;
            else:
                set_to_source_flag(flag=False);
                set_to_dest_flag(flag=True);
                set_returning_flag(flag=False);
                self.command_navigation_with_path();
                self.control_report_publish(control=control, control_type="control",response_code=201);
                response.result = True;
        elif control_code == CONTROL_CODE_MS_COMPLETE:
            if get_to_dest_flag() is True or get_to_source_flag() is True:
                self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Robot is Moving To Dest");
                self.control_report_publish(control=control, control_type="control", response_code=400);
                response.result = False;
            else:
                set_to_source_flag(flag=False);
                set_to_dest_flag(flag=False);
                set_returning_flag(flag=True);
                self.command_navigation_with_path();
                self.control_report_publish(control=control, control_type="control", response_code=201);
                response.result = True;
        elif control_code == CONTORL_CODE_GRAPH_SYNC:
            graph_sync_result: Graph.Response = self.graph_sync_request();

            if graph_sync_result is not None:
                try:
                    graph_list: GraphList = message_conversion.populate_instance(msg=json.loads(graph_sync_result.graph_list), inst=GraphList());
                    self.__log.info(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} Graph List\n{ros_message_dumps(message=graph_list)}");
                    self.__graph_list_publisher.publish(msg=graph_list);
                    control_report_data: dict = {
                        "map_id": graph_list.graph[0].map_id,
                        "version": graph_list.graph[0].version
                    };
                    self.control_report_publish(control=control, control_type="graph", response_code=201, control_data=control_report_data);
                except message_conversion.NonexistentFieldException as nefe:
                    self.__log.error(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} : {nefe}");

                response.result = True;
            else:
                response.result = False;
        else:
            self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Unknown Control Code...");
            response.result = False;
        return response;

    def control_report_publish(self, control: Control, control_type: str, response_code: int, control_data: dict | None = None) -> None:
        try:
            control_report: ControlReport = ControlReport();
            control_report.create_time = get_current_time();
            control_report.control_id = control.control_id;
            control_report.control_code = control.control_code;
            control_report.response_code = response_code;

            if control_type == "control":
                control_report.control_type = control_type;
            elif control_type == "graph":
                control_report.control_type = "";

                control_report_data_graph_list: list[ControlReportDataGraphList] = [];
                control_report_data_graph_list.append(message_conversion.populate_instance(msg=control_data, inst=ControlReportDataGraphList()));
                control_report.data.graph_list = control_report_data_graph_list;

            self.__control_report_publisher.publish(msg=control_report);
        except TypeError as te:
            self.__log.error(f"{NOTIFY_CONTROL_REPORT_TOPIC_NAME} : {te}");
            return;

    def command_navigation_with_path(self) -> None:
        self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} Current Drive Status : {self.__drive_status}");

        global to_source_flag;
        global to_dest_flag;
        global returning_flag;

        if self.__mission is None:
            self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} mission is None");
            set_to_source_flag(flag=False);
            set_to_dest_flag(flag=False);
            set_returning_flag(flag=False);
            return;

        mission_task: MissionTask = self.__mission.task[0];
        self.__log.info(
            f"{PATH_GRAPH_PATH_SERVICE_NAME} Request Task\n{ros_message_dumps(message=mission_task)}");

        self.__log.info(
            f"{PATH_GRAPH_PATH_SERVICE_NAME} Current Flags\n\tto_source_flag : {to_source_flag}\n\tto_dest_flag : {to_dest_flag}\n\treturn_flag : {returning_flag}\n\tublox : {self.__ublox_fix is None}");

        path_graph_path_request: Path.Request = Path.Request();

        if self.__ublox_fix is not None:
            if to_source_flag is True and to_dest_flag is False and returning_flag is False:
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

                self.__log.info(
                    f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path Request is ready\n{ros_message_dumps(message=path_graph_path_request)}");

                is_path_graph_path_service_server_ready: bool = self.__path_graph_path_service_client.wait_for_service(timeout_sec=1.0);

                if is_path_graph_path_service_server_ready:
                    to_source_request_response: Path.Response = self.__path_graph_path_service_client.call(request=path_graph_path_request);

                    if to_source_request_response is None:
                        self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path Response is None...");
                        return;

                    self.__path_response = to_source_request_response;
                    self.__path_response_list_size = len(self.__path_response.path.node_list);

                    self.__log.info(
                        f"{PATH_GRAPH_PATH_SERVICE_NAME} To Source Path response : {ros_message_dumps(message=to_source_request_response)}, size : {self.__path_response_list_size}");

                    """
                    #######################################
                    대기 장소 -> 상차지(출발지) 주행 첫 번째 Goal 송신
                    #######################################
                    """
                    self.route_to_pose_send_goal();
                else:
                    self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} is not ready...");
                    return;

            elif to_dest_flag is True and to_source_flag is False and returning_flag is False:
                if self.__mission is None:
                    self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Dest Mission is None...");
                    return;
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

                self.__log.info(
                    f"{PATH_GRAPH_PATH_SERVICE_NAME} To Goal Path Request is ready : {ros_message_dumps(message=path_graph_path_request)}");

                is_path_graph_path_service_server_ready: bool = self.__path_graph_path_service_client.wait_for_service(timeout_sec=1.0);

                if is_path_graph_path_service_server_ready:
                    to_dest_path_response: Path.Response = self.__path_graph_path_service_client.call(request=path_graph_path_request);

                    if to_dest_path_response is None:
                        self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} To Dest Path Response is None...");
                        return;

                    self.__path_response = to_dest_path_response;
                    self.__path_response_list_size = len(self.__path_response.path.node_list);

                    self.__log.info(
                        f"{PATH_GRAPH_PATH_SERVICE_NAME} To Dest Path response : {ros_message_dumps(message=to_dest_path_response)}, size : {self.__path_response_list_size}");

                    """
                    #######################################
                    상차지(출발지) -> 하차지 주행 첫 번째 Goal 송신
                    #######################################
                    """
                    self.route_to_pose_send_goal();
            elif returning_flag is True and to_source_flag is False and to_dest_flag is False:
                if self.__mission is None:
                    self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} Return Mission is None...");
                    return;
                self.notify_mission_status_publish(status="End");
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

                self.__log.info(
                    f"{PATH_GRAPH_PATH_SERVICE_NAME} Return Path Request is ready : {ros_message_dumps(message=path_graph_path_request)}");

                is_path_graph_path_service_server_ready: bool = self.__path_graph_path_service_client.wait_for_service(timeout_sec=1.0);

                if is_path_graph_path_service_server_ready:
                    returning_path_response: Path.Response = self.__path_graph_path_service_client.call(request=path_graph_path_request);

                    if returning_path_response is None:
                        self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} Returning Path Response is None...");
                        return;

                    self.__path_response = returning_path_response;
                    self.__path_response_list_size = len(self.__path_response.path.node_list);

                    self.__log.info(
                        f"{PATH_GRAPH_PATH_SERVICE_NAME} Returning Path response : {ros_message_dumps(message=returning_path_response)}, size : {self.__path_response_list_size}");

                    """
                    #######################################
                    하차지 -> 상차지(출발지) 주행 첫 번째 Goal 송신
                    #######################################
                    """
                    self.route_to_pose_send_goal();
            else:
                self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} Unknown Flag");
                return;
        else:
            self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} GPS Is None...");
            return;

    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        self.__ublox_fix = ublox_fix_cb;

        if self.__ublox_fix is None:
            error_report: String = String();
            error_report.data = "451";
            self.__error_report_publisher.publish(msg=error_report);

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
                    self.notify_mission_status_publish(status="Started");
                    self.__log.info(f"==================================== Started ====================================");
            elif to_dest_flag is True and to_source_flag is False and returning_flag is False:
                if self.__route_to_pose_goal_index == 0:
                    """
                    상차지(출발지) -> 하차지 주행 출발 시
                    """
                    self.notify_mission_status_publish(status="OnProgress");
            else:
                return;
        elif status_code == 5001:  # 취소
            # 주행 취소 : 3
            self.__drive_status = DRIVE_STATUS_CANCELLED;
            self.notify_mission_status_publish(status="Cancelled");

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
                    self.notify_mission_status_publish(status="SourceArrived");
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;

                    self.__log.info(f"==================================== Source Arrived ====================================");
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
                    self.notify_mission_status_publish(status="DestArrived");
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;

                    self.__log.info(f"==================================== Dest Arrived ====================================");
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
                    self.__drive_status = DRIVE_STATUS_WAIT;
                    self.__drive_current = ("", "");

                    self.__route_to_pose_goal_index = 0;
                    self.__path_response = None;
                    self.__path_response_list_size = 0;
                    self.__mission = None;
                    to_source_flag = False;
                    to_dest_flag = False;
                    returning_flag = False;
                    self.__log.info(f"==================================== Returning Finished ====================================");
                else:
                    self.__route_to_pose_goal_index = self.__route_to_pose_goal_index + 1;
                    self.__log.info(f"{ROUTE_TO_POSE_ACTION_NAME} Returning will proceed Next Goal [{self.__route_to_pose_goal_index}]");
                    self.route_to_pose_send_goal();
            else:
                return;
        elif result_code == 2001:  # 실패
            self.notify_mission_status_publish(status="Failed");

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

        self.__log.info(
            f"{NOTIFY_MISSION_STATUS_TOPIC_NAME} Service Status\n{ros_message_dumps(message=service_status)}");
        self.__notify_mission_status_publisher.publish(msg=service_status);

    def notify_navigation_status_publish(self) -> None:
        status: Status = Status();

        status.map_id = self.__path_response.map_id;
        status.drive_status = self.__drive_status;

        status.from_node = self.__drive_current[0];
        status.to_node = self.__drive_current[1];

        self.__notify_navigation_status_publisher.publish(msg=status);

    def notify_mission_timer_cb(self) -> None:
        if self.__path_response is not None:
            self.notify_navigation_status_publish();

    def graph_sync_request(self) -> Graph.Response | None:
        graph_request: Graph.Request = Graph.Request();
        graph_request.send_id = f"{KTP_DEVICE_ID}{get_current_time()}";

        is_path_graph_graph_service_server_ready: bool = self.__path_graph_graph_service_client.wait_for_service(timeout_sec=0.8);

        if is_path_graph_graph_service_server_ready:
            graph_response: Graph.Response = self.__path_graph_graph_service_client.call(request=graph_request);

            self.__log.info(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} Graph Response\n{ros_message_dumps(message=graph_response)}");

            return graph_response;
        else:
            self.__log.error(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} Service Server is Not Ready...");
            return None;

    def error_report_publish(self, error_code: str) -> None:
        std_string: String = String();
        std_string.data = error_code;
        self.__error_report_publisher.publish(msg=std_string);


__all__ = ["Processor", "set_to_source_flag", "set_to_dest_flag", "set_returning_flag"];
