import json;
import time;
from rclpy.node import Node;
from rclpy.client import Client;
from rclpy.service import Service;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.msg import ControlReport;
from ktp_data_msgs.srv import AssignControl;
from ktp_data_msgs.msg import GraphList;
from path_graph_msgs.srv import Path;
from path_graph_msgs.srv import Graph;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.application.path import PathService;
from ktp_task_controller.application.status import StatusService;
from ktp_task_controller.application.route import RouteService;
from ktp_task_controller.utils import ros_message_dumps;
from ktp_task_controller.utils import get_current_time;
from ktp_task_controller.utils import get_initial_node_id;
from ktp_task_controller.domain.flags import get_driving_flag;
from ktp_task_controller.domain.mission import get_mission;
from ktp_task_controller.domain.status import set_is_mission_canceled;
from ktp_task_controller.domain.status import set_driving_status;
from typing import Any;


ASSIGN_CONTROL_SERVICE_NAME: str = "/ktp_task_controller/assign/control";
NOTIFY_CONTROL_REPORT_TOPIC_NAME: str = "/rms/ktp/task/notify/control/report";
PATH_GRAPH_GRAPH_SERVICE_NAME: str = "/path_graph_msgs/graph";
GRAPH_LIST_TOPIC: str = "/rms/ktp/task/notify/graph_list";

CONTROL_CODE_STOP: str = "stop";
CONTROL_CODE_RELEASE: str = "release";
CONTROL_MS_CANCEL: str = "mscancel";
CONTROL_CODE_MOVE_TO_DEST: str = "movetodest";
CONTROL_CODE_MS_COMPLETE: str = "mscomplete";
CONTORL_CODE_GRAPH_SYNC: str = "graphsync";

DRIVE_STATUS_WAIT: int = 0;
DRIVE_STATUS_CANCELLED: int = 3;

class ControlController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.__param_dev_id: str = self.__node.get_parameter(name="dev_id").get_parameter_value().string_value;
        self.__param_map_id: str = self.__node.get_parameter(name="map_id").get_parameter_value().string_value;
        
        self.__error_service: ErrorService = ErrorService(node=self.__node);
        self.__path_service: PathService = PathService(node=self.__node);
        self.__status_service: StatusService = StatusService(node=self.__node);
        self.__route_service: RouteService = RouteService(node=self.__node);
        
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
        
        graph_list_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_publisher: Publisher = self.__node.create_publisher(
            topic=GRAPH_LIST_TOPIC,
            msg_type=GraphList,
            callback_group=graph_list_publisher_cb_group,
            qos_profile=qos_profile_system_default
        );
        
        graph_sync_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_sync_client: Client = self.__node.create_client(
            srv_name=PATH_GRAPH_GRAPH_SERVICE_NAME,
            srv_type=Graph,
            callback_group=graph_sync_service_client_cb_group,
            qos_profile=qos_profile_services_default
        );
        
    def assign_control_service_cb(self, request: AssignControl.Request, response: AssignControl.Response) -> AssignControl.Response:
        try:
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
                """
                Cancel Mission
                """
                set_is_mission_canceled(is_mission_canceled=True);
                set_driving_status(driving_status=DRIVE_STATUS_CANCELLED);
                self.control_report_publish(control=control, control_type="control", response_code=201);
                self.__status_service.notify_mission_status_publish(status="Cancelled");
                self.__log.info(f"==================================== Cancelled ====================================");
                self.__log.info(f"==================================== SLEEP ====================================");
                time.sleep(4.0);
                set_driving_status(driving_status=DRIVE_STATUS_WAIT);
            elif control_code == CONTROL_CODE_MOVE_TO_DEST:
                """
                Source -> Goal
                - start_node : task.task_data.source
                - end_node : task.task_data.goal
                """
                path_request: Path.Request = Path.Request();

                mission: Mission = get_mission();
                if mission != None:
                    path_request.start_node = mission.task[0].task_data.source;
                    path_request.end_node = mission.task[0].task_data.goal[0];
                    path_response: Path.Response = self.__path_service.convert_path_request(path_request=path_request);
                        
                    if path_response != None:
                        self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} Path Response\n{ros_message_dumps(message=path_response)}");
                            
                        if get_driving_flag() != True:
                            self.__route_service.send_goal(path_response=path_response);
                            response.result = True;
                            self.control_report_publish(control=control, control_type="control", response_code=201);
                        else:
                            self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} is already driving");
                            response.result = False;
                            self.control_report_publish(control=control, control_type="control", response_code=400);
                            self.__error_service.error_report_publish(error_code="999");
                    else:
                        self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Path Response is None");
                        self.control_report_publish(control=control, control_type="control", response_code=400);
                        self.__error_service.error_report_publish(error_code="999");
                        response.result = False;
                else:
                    self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} mission is None");
                    self.control_report_publish(control=control, control_type="control", response_code=400);
                    self.__error_service.error_report_publish(error_code="999");
                    response.result = False;
                        
                return response;
            elif control_code == CONTROL_CODE_MS_COMPLETE:
                self.__route_service.end_mission();
                        
                control_data: Any = message_conversion.extract_values(inst=control.control_data);
                is_return: bool = control_data["is_return"];
                self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} MSCOMPLETE is_return : {is_return}");
                
                if is_return:
                    """
                    Goal -> 대기 장소
                    - start_node : task.task_data.goal
                    - end_node : initial_node_id
                    """
                    path_request: Path.Request = Path.Request();

                    mission: Mission = get_mission();
                    if mission != None:
                        initial_node_id: str = f"NO-{self.__param_map_id}-{get_initial_node_id(log=self.__log)}";
                        
                        path_request.start_node = mission.task[0].task_data.goal[0];
                        path_request.end_node = initial_node_id;
                        
                        path_response: Path.Response = self.__path_service.convert_path_request(path_request=path_request);

                        if path_response != None:
                            self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} Path Response\n{ros_message_dumps(message=path_response)}");
                                
                            if get_driving_flag() != True:
                                self.__route_service.send_goal(path_response=path_response);
                                response.result = True;
                                self.control_report_publish(control=control, control_type="control", response_code=201);
                            else:
                                self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} is already driving");
                                response.result = False;
                                self.control_report_publish(control=control, control_type="control", response_code=400);
                                self.__error_service.error_report_publish(error_code="999");
                        else:
                            self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Path Response is None");                            
                            response.result = False;
                            self.control_report_publish(control=control, control_type="control", response_code=400);
                            self.__error_service.error_report_publish(error_code="999");
                    else:
                        self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} mission is None");
                        self.control_report_publish(control=control, control_type="control", response_code=400);
                        self.__error_service.error_report_publish(error_code="999");
                        response.result = False;
                else:
                    """
                    대기 장소 미복귀
                    """
                    self.__log.info(f"{ASSIGN_CONTROL_SERVICE_NAME} MSCOMPLETE No Return");
                    
                    if get_mission() is None:
                        self.control_report_publish(control=control, control_type="control", response_code=400);
                        response.result = False;
                    else:
                        self.control_report_publish(control=control, control_type="control", response_code=201);
                        response.result = True;
                        
                    self.__route_service.process_no_return();
                    
                return response;
            elif control_code == CONTORL_CODE_GRAPH_SYNC:
                graph_sync_result: Graph.Response = self.graph_sync_request();

                if graph_sync_result is not None:
                    try:
                        graph_list: GraphList = message_conversion.populate_instance(msg=json.loads(graph_sync_result.graph_list), inst=GraphList());
                        graph_list.send_id = control.control_id;
                        self.__log.info(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} Graph List\n{ros_message_dumps(message=graph_list)}");
                        self.__graph_list_publisher.publish(msg=graph_list);
                        control_report_data: dict = {
                            "map_id": graph_list.graph[0].map_id,
                            "version": graph_list.graph[0].version
                        };
                        self.control_report_publish(control=control, control_type="graph", response_code=201, control_data=control_report_data);
                        response.result = True;
                    except message_conversion.NonexistentFieldException as nefe:
                        self.__log.error(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} : {nefe}");
                        self.control_report_publish(control=control, control_type="graph", response_code=400, control_data=control_report_data);
                        self.__error_service.error_report_publish(error_code="999");
                        response.result = False;
                else:
                    self.control_report_publish(control=control, control_type="graph", response_code=400, control_data=control_report_data);
                    self.__error_service.error_report_publish(error_code="999");
                    response.result = False;
            else:
                self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} Unknown Control Code...");
                self.control_report_publish(control=control, control_type="graph", response_code=400, control_data=control_report_data);
                self.__error_service.error_report_publish(error_code="999");
                response.result = False;
                
            return response;
        except Exception as e:
            self.__log.error(f"{ASSIGN_CONTROL_SERVICE_NAME} : {e}");
            self.control_report_publish(control=control, control_type="control", response_code=400);
            self.__error_service.error_report_publish(error_code="999");
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
                control_report.control_type = "control";

            self.__control_report_publisher.publish(msg=control_report);
        except TypeError as te:
            self.__log.error(f"{NOTIFY_CONTROL_REPORT_TOPIC_NAME} : {te}");
            return;
    
    def graph_sync_request(self) -> Graph.Response | None:
        graph_request: Graph.Request = Graph.Request();
        graph_request.send_id = f"{self.__param_dev_id}{get_current_time()}";

        is_path_graph_graph_service_server_ready: bool = self.__graph_sync_client.wait_for_service(timeout_sec=0.8);

        if is_path_graph_graph_service_server_ready:
            graph_response: Graph.Response = self.__graph_sync_client.call(request=graph_request);

            self.__log.info(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} Graph Response\n{ros_message_dumps(message=graph_response)}");

            return graph_response;
        else:
            self.__log.error(f"{PATH_GRAPH_GRAPH_SERVICE_NAME} Service Server is Not Ready...");
            self.__error_service.error_report_publish(error_code="999");
            return None;

        
__all__: list[str] = ["ControlController"];