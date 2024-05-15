from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.service import Service;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.srv import AssignControl;
from ktp_data_manager.application.control import ControlService;


ASSIGN_CONTROL_FROM_ITF_SERVICE_NAME: str = "/ktp_data_manager/assign/control";


class ControlController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__control_service: ControlService = ControlService(node=self.__node);
        
        assign_control_from_itf_service_server_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_from_itf_service_server: Service = self.__node.create_service(
            srv_name=ASSIGN_CONTROL_FROM_ITF_SERVICE_NAME,
            srv_type=AssignControl,
            qos_profile=qos_profile_services_default,
            callback_group=assign_control_from_itf_service_server_cb_group,
            callback=self.assign_control_from_itf_service_server_cb
        );
    
    def assign_control_from_itf_service_server_cb(self, request: AssignControl.Request, response: AssignControl.Response) -> AssignControl.Response:
        if request.control.control_code == "" or None:
            self.__log.error(f"{ASSIGN_CONTROL_FROM_ITF_SERVICE_NAME} control_code is empty");
            response.result = False;
        else:
            is_assign_succeeded: bool = self.__control_service.assign_cotrol_request(request=request);
            
            if is_assign_succeeded:
                response.result = True;
            else:
                self.__log.error(f"{ASSIGN_CONTROL_FROM_ITF_SERVICE_NAME} failed to assign control");
                response.result = False;
                
        return response;
    
    
__all__: list[str] = ["ControlController"];