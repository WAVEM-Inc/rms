from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.service import Service;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.srv import AssignMission;
from ktp_data_manager.application.mission import MissionService;


ASSIGN_MISSION_FROM_ITF_SERVICE_NAME: str = "/ktp_data_manager/assign/mission";


class MissionController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__mission_service: MissionService = MissionService(node=self.__node);
        
        assign_mission_from_itf_service_server_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_mission_from_itf_service_server: Service = self.__node.create_service(
            srv_name=ASSIGN_MISSION_FROM_ITF_SERVICE_NAME,
            srv_type=AssignMission,
            qos_profile=qos_profile_services_default,
            callback_group=assign_mission_from_itf_service_server_cb_group,
            callback=self.assign_mission_from_itf_service_server_cb
        );
        
    def assign_mission_from_itf_service_server_cb(self, request: AssignMission.Request, response: AssignMission.Response) -> AssignMission.Response:
        is_assign_succeeded: bool = self.__mission_service.assign_mission_request(request=request);
            
        if is_assign_succeeded:
            response.result = True;
        else:
            self.__log.error(f"{ASSIGN_MISSION_FROM_ITF_SERVICE_NAME} failed to assign mission");
            response.result = False;
                
        return response;
        
        
__all__: list[str] = ["MissionController"];