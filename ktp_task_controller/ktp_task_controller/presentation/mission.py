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
from ktp_task_controller.application.path import PathService;
from ktp_task_controller.application.route import RouteService;
from ktp_task_controller.domain.flags import set_to_source_flag;
from ktp_task_controller.domain.flags import set_to_goal_flag;
from ktp_task_controller.domain.flags import set_returning_flag;
from ktp_task_controller.domain.flags import get_driving_flag;
from ktp_task_controller.domain.mission import get_mission;
from ktp_task_controller.domain.mission import set_mission;


NODE_NAME: str = "ktp_task_controller";
ASSIGN_MISSION_TOPIC_NAME: str = "/rms/ktp/data/assign/mission";
ASSIGN_MISSION_SERVICE_NAME: str = f"/{NODE_NAME}/assign/mission";


class MissionController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.__path_service: PathService = PathService(node=self.__node);
        self.__route_service: RouteService = RouteService(node=self.__node);
        
        assign_mission_service_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_service: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_SERVICE_NAME,
            srv_type=AssignMission,
            callback_group=assign_mission_service_cb_group,
            callback=self.assign_mission_service_cb,
            qos_profile=qos_profile_services_default
        );
        
    def assign_mission_service_cb(self, request: AssignMission.Request, response: AssignMission.Response) -> AssignMission.Response:
        request_mission_json: str = ros_message_dumps(message=request.mission);
        self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} request\n{request_mission_json}");

        if get_mission() is not None:
            self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Mission has already scheduled...");
            response.result = False;
        else:
            mission = message_conversion.populate_instance(json.loads(request_mission_json), Mission());

            set_to_source_flag(flag=True);
            set_to_goal_flag(flag=False);
            set_returning_flag(flag=False);
            set_mission(mission=mission);
            
            mission_task: MissionTask = get_mission().task[0];
            self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Path Request\n{ros_message_dumps(message=mission_task)}");

            path_request: Path.Request = Path.Request();
            path_request.position.longitude = 0.0;
            path_request.position.latitude = 0.0;

            path_request.start_node = "";
            path_request.end_node = mission_task.task_data.source;
            
            path_response: Path.Response = self.__path_service.convert_path_request(path_request=path_request);
            if path_response != None:
                self.__log.info(f"{ASSIGN_MISSION_SERVICE_NAME} Path Response\n{ros_message_dumps(message=path_response)}");
                
                if get_driving_flag() != True:
                    self.__route_service.send_goal(path_response=path_response);
                    response.result = True;
                else:
                    self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} is already driving");
                    response.result = False;
                    return;
            else:
                self.__log.error(f"{ASSIGN_MISSION_SERVICE_NAME} Path Response is None");
                self.__route_service.goal_flush();
                response.result = False;
                
        return response;
    

__all__: list[str] = ["MissionController"];