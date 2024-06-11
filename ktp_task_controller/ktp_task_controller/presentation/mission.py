import time;
import json;
from rclpy.node import Node;
from rclpy.service import Service;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_services_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rosbridge_library.internal import message_conversion;
from ktp_data_msgs.msg import Mission;
from ktp_data_msgs.msg import MissionTask;
from ktp_data_msgs.srv import AssignMission;
from path_graph_msgs.srv import Path;
from ktp_task_controller.utils import ros_message_dumps;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.application.path import PathService;
from ktp_task_controller.application.route import RouteService;
from ktp_task_controller.application.status import StatusService;
from ktp_task_controller.domain.flags import get_driving_flag;
from ktp_task_controller.domain.mission import get_mission;
from ktp_task_controller.domain.mission import set_mission;
from ktp_task_controller.domain.status import get_last_arrived_node_id;
from ktp_task_controller.domain.status import set_driving_status;


NODE_NAME: str = "ktp_task_controller";
ASSIGN_MISSION_TOPIC_NAME: str = "/rms/ktp/data/assign/mission";
ASSIGN_MISSION_SERVICE_NAME: str = f"/{NODE_NAME}/assign/mission";

DRIVE_STATUS_DRIVE_FINISHED: int = 2;


class MissionController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.__param_map_id: str = self.__node.get_parameter(name="map_id").get_parameter_value().string_value;
        self.__param_initial_node: str = self.__node.get_parameter(name="initial_node").get_parameter_value().string_value;
        
        self.__error_service: ErrorService = ErrorService(node=self.__node);
        self.__path_service: PathService = PathService(node=self.__node);
        self.__route_service: RouteService = RouteService(node=self.__node);
        self.__status_service: StatusService = StatusService(node=self.__node);
        
        assign_mission_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            callback_group=assign_mission_service_cb_group,
            callback=self.assign_mission_service_cb,
            qos_profile=qos_profile_services_default
        );
        
    def assign_mission_service_cb(self, request: AssignMission.Request, response: AssignMission.Response) -> AssignMission.Response:
        try:
            request_mission_json: str = ros_message_dumps(message=request.mission);
            self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} request\n{request_mission_json}");

            if get_mission() is not None:
                self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Mission has already scheduled...");
                response.result = False;
            else:
                mission = message_conversion.populate_instance(json.loads(request_mission_json), Mission());
                set_mission(mission=mission);
                
                mission_task: MissionTask = get_mission().task[0];
                self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Path Request\n{ros_message_dumps(message=mission_task)}");
                            
                path_request: Path.Request = Path.Request();
                            
                source_node_id: str = mission_task.task_data.source;
                goal_node_id: str = mission_task.task_data.goal[0];
                last_arrived_node_id_is_source: bool = get_last_arrived_node_id() == source_node_id;
                self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} mission_assignment"
                                f"\n\tsource_node_id : {source_node_id}"
                                f"\n\tlast_arrived_node_id : {get_last_arrived_node_id()}"
                                f"\n\tlast_arrived_node_id_is_source : {last_arrived_node_id_is_source}");
                
                is_mission_returning_task: bool = mission_task.task_code == "returning";
                
                if source_node_id == "":
                    path_request.start_node = get_last_arrived_node_id();
                    path_request.end_node = goal_node_id;  
                elif last_arrived_node_id_is_source:
                    path_request.start_node = source_node_id;
                    path_request.end_node = goal_node_id;
                else:
                    if not is_mission_returning_task:
                        self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Mission Is Wait To Source");
                        path_request.start_node = get_last_arrived_node_id();
                        path_request.end_node = source_node_id;
                    else:
                        self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Mission Is Returning Task");
                        path_request.start_node = source_node_id;
                        path_request.end_node = goal_node_id;
                
                path_response: Path.Response = self.__path_service.convert_path_request(path_request=path_request);
                
                if path_response != None or len(path_response.path.node_list) != 0:
                    self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Path Response\n{ros_message_dumps(message=path_response)}");
                    
                    if get_driving_flag() != True:
                        self.__route_service.start_mission();
                        self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Sleep For Assign Mission");
                        time.sleep(2.0);
                        self.__log.info(f"==================================== SLEEP ====================================");
                        if last_arrived_node_id_is_source:
                            if not is_mission_returning_task:
                                set_driving_status(driving_status=DRIVE_STATUS_DRIVE_FINISHED);
                                self.__status_service.notify_mission_status_publish(status="SourceArrived");
                                self.__log.info(f"==================================== Source Arrived ====================================");
                                self.__log.info(f"==================================== Wait For MSCOMPLETE ====================================");
                                response.result = True;
                            else:
                                self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Mission Is Returning Task");
                                self.__log.info(f"==================================== SLEEP ====================================");
                                pass;
                        else:
                            time.sleep(2.0);
                            self.__route_service.send_goal(path_response=path_response);
                            response.result = True;
                    else:
                        self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} is already driving");
                        response.result = False;
                else:
                    self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Path Response is None");
                    self.__error_service.error_report_publish(error_code="201");
                    self.__status_service.notify_mission_status_publish(status="Failed");
                    self.__route_service.goal_flush();
                    self.__route_service.mission_flush();
                    response.result = False;
                    return response;
                    
            return response;
        except Exception as e:
            self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} : {e}");
            self.__error_service.error_report_publish(error_code="999");
            self.__status_service.notify_mission_status_publish(status="Failed");
            response.result = False;
            return response;
    

__all__: list[str] = ["MissionController"];