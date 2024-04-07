import json;
from rclpy.node import Node;
from rclpy.service import Service;
from rclpy.client import Client;
from rclpy.subscription import Subscription;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rosbridge_library.internal import message_conversion;
from sensor_msgs.msg import NavSatFix;
from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_msgs.msg import ServiceStatusTask;
from ktp_data_msgs.msg import ServiceStatusTaskData;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.msg import MissionTask;
from ktp_data_msgs.msg import MissionTaskData;
from ktp_data_msgs.srv import AssignMission;
from ktp_task_controller.application.impl.mission import MissionProcessor;
from ktp_task_controller.application.impl.navigation import NavigationProcessor;
from ktp_task_controller.internal.utils import ros_message_dumps;
from ktp_task_controller.internal.utils import get_current_time;
from ktp_task_controller.internal.constants import (
    ASSIGN_MISSION_SERVICE_NAME,
    UBLOX_FIX_TOPIC_NAME,
    NOTIFY_MISSION_STATUS_TOPIC_NAME,
    CONVERT_TASK_TO_PATH_SERVICE_NAME
);
from ktp_task_msgs.srv import ConvertTaskToPath;


class MissionProcessor:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();

        self.__mission: Mission = None;
        self.__ublox_fix: NavSatFix = None;
    
        assign_mission_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            callback_group=assign_mission_service_cb_group,
            callback=self.assign_mission_service_cb,
            qos_profile=qos_profile_services_default
        );
        
        convert_task_to_path_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__convert_task_to_path_service_client: Client = self.__node.create_client(
            srv_name=CONVERT_TASK_TO_PATH_SERVICE_NAME,
            srv_type=ConvertTaskToPath,
            callback_group=convert_task_to_path_service_client_cb_group,
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

    def assign_mission_service_cb(self, request: AssignMission.Request, response: AssignMission.Response) -> AssignMission.Response:
        request_mission_json: str = ros_message_dumps(message=request.mission);
        self.__log.info(message=f"{ASSIGN_MISSION_SERVICE_NAME} request\n{request_mission_json}");

        if self.__mission is not None:
            self.__log.error(message=f"{ASSIGN_MISSION_SERVICE_NAME} Mission has already scheduled...");
            response.result = False;
        else:
            try:
                self.__mission = message_conversion.populate_instance(msg=json.loads(s=request_mission_json), inst=Mission());

                if self.__ublox_fix is None:
                    self.__log.error(message=f"{ASSIGN_MISSION_SERVICE_NAME} {UBLOX_FIX_TOPIC_NAME} is None...");
                    request_mission_json = "";
                    self.__mission = None;
                    response.result = False;
                    # self.error_report_publish(error_code="451");
                else:
                    is_task_to_path_conversion_succeeded: bool = self.convert_task_to_path_service_request();
                    
                    if is_task_to_path_conversion_succeeded:
                        response.result = True;
                    else:
                        response.result = False;
            except message_conversion.NonexistentFieldException as nefe:
                self.__log.error(message=f"{ASSIGN_MISSION_SERVICE_NAME} : {nefe}");
                response.result = False;

        return response;
    
    def convert_task_to_path_service_request(self) -> bool:
        convert_task_to_path_request: ConvertTaskToPath.Request = ConvertTaskToPath.Request();
        convert_task_to_path_request.task = self.__mission.task[0];
        
        is_convert_task_to_path_service_server_ready: bool = self.__convert_task_to_path_service_client.wait_for_service(timeout_sec=0.8);
        
        if is_convert_task_to_path_service_server_ready:
            path_response: ConvertTaskToPath.Response = self.__convert_task_to_path_service_client.call(request=convert_task_to_path_request);
            
            is_path_response_succeeded: bool = path_response.result;
            
            if is_path_response_succeeded:
                self.__log.info(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} request succeeded");
                return True;
            else:
                self.__log.error(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} request failed");
                return False;
        else:
            self.__log.error(message=f"{CONVERT_TASK_TO_PATH_SERVICE_NAME} service server is not ready...");
            return False;

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
            message=f"{NOTIFY_MISSION_STATUS_TOPIC_NAME} Service Status\n{ros_message_dumps(message=service_status)}");
        self.__notify_mission_status_publisher.publish(msg=service_status);

    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        self.__ublox_fix = ublox_fix_cb;

        if self.__ublox_fix is None:
            # error_report: String = String();
            # error_report.data = "451";
            # self.__error_report_publisher.publish(msg=error_report);
            pass;
    
__all__: list[str] = ["MissionProcessor"];