from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.client import Client;
from rclpy.qos import qos_profile_services_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.srv import AssignControl;


ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME: str = "/ktp_task_controller/assign/control";


class ControlService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        assign_control_to_task_ctrl_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__assign_control_to_task_ctrl_service_client: Client = self.__node.create_client(
            srv_name=ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME,
            srv_type=AssignControl,
            qos_profile=qos_profile_services_default,
            callback_group=assign_control_to_task_ctrl_service_client_cb_group
        );
    
    def assign_cotrol_request(self, request: AssignControl.Request) -> bool:
        request_result: bool = False;
        try:
            is_assign_control_service_server_ready: bool = self.__assign_control_to_task_ctrl_service_client.wait_for_service(timeout_sec=0.7);
            
            if is_assign_control_service_server_ready:
                response: AssignControl.Response = self.__assign_control_to_task_ctrl_service_client.call(request=request);
                self.__log.info(f"{ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME} result : {response.result}");
                request_result = response.result;
            else:
                self.__log.error(f"{ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME} service server is not ready");
                request_result = False;
            
            return request_result;
        except AttributeError as ate:
            self.__log.error(f"{ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME} : {ate}");
            request_result = False;
            
            return request_result;
    
        
__all__: list[str] = ["ControlService"];