import json;
from rclpy.node import Node;
from rclpy.service import Service;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.msg import ControlReport;
from ktp_data_msgs.srv import AssignControl;
from ktp_task_controller.application.utils import ros_message_dumps;
from ktp_task_controller.application.utils import get_current_time;
from ktp_task_controller.application.mission_controller import MissionController;
from ktp_task_controller.application.mission_controller import set_to_source_flag;
from ktp_task_controller.application.mission_controller import set_to_dest_flag;
from ktp_task_controller.application.mission_controller import set_returning_flag;


NODE_NAME: str = "ktp_task_controller";
ASSIGN_CONTROL_SERVICE_NAME: str = f"/{NODE_NAME}/assign/control";
PATH_GRAPH_PATH_SERVICE_NAME: str = "/path_graph_msgs/path";
UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
NOTIFY_MISSION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/mission/status";
NOTIFY_CONTROL_REPORT_TOPIC_NAME: str = "/rms/ktp/task/notify/control/report";
NOTIFY_NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/navigation/status";
NOTIFY_CONTROL_READY_TOPIC_NAME: str = "/rms/ktp/task/notify/control/ready";

CONTROL_CODE_STOP: str = "stop";
CONTROL_CODE_RELEASE: str = "release";
CONTROL_MS_CANCEL: str = "mscancel";
CONTROL_CODE_MOVE_TO_DEST: str = "movetodest";
CONTROL_CODE_MS_COMPLETE: str = "mscomplete";


class ControlController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mission_controller: MissionController = MissionController(node=self.__node);
        
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
            set_to_source_flag(flag=False);
            set_to_dest_flag(flag=True);
            set_returning_flag(flag=False);
            self.__mission_controller.command_navigation_with_path();
            self.control_report_publish(control=control, response_code=201);
        elif control_code == CONTROL_CODE_MS_COMPLETE:
            set_to_source_flag(flag=False);
            set_to_dest_flag(flag=False);
            set_returning_flag(flag=True);
            self.__mission_controller.command_navigation_with_path();
            self.control_report_publish(control=control, response_code=201);

        response.result = True;

        return response;

    def control_report_publish(self, control: Control, response_code: int) -> None:
        control_report: ControlReport = ControlReport();
        control_report.create_time = get_current_time();
        control_report.control_id = control.control_id;
        control_report.control_type = "control";
        control_report.control_code = control.control_code;
        control_report.response_code = response_code;

        self.__control_report_publisher.publish(msg=control_report);


__all__ = ["ControlController"];