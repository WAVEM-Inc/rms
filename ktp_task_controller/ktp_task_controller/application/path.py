from rclpy.node import Node;
from rclpy.client import Client;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from path_graph_msgs.srv import Path;
from path_graph_msgs.srv import Graph;
import route_msgs.msg as route;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.utils import ros_message_dumps;

PATH_GRAPH_PATH_SERVICE_NAME: str = "/path_graph_msgs/path";
PATH_GRAPH_GRAPH_SERVICE_NAME: str = "/path_graph_msgs/graph";
GRAPH_LIST_TOPIC: str = "/rms/ktp/task/notify/graph_list";
NOTIFY_PATH_TOPIC: str = "/rms/ktp/task/notify/path";


class PathService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__error_service: ErrorService = ErrorService(node=self.__node);
        
        self.__path_graph_path_service_client: Client = None;
        if self.__path_graph_path_service_client is None:
            path_graph_path_service_client_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__path_graph_path_service_client = self.__node.create_client(
                srv_name=PATH_GRAPH_PATH_SERVICE_NAME,
                srv_type=Path,
                callback_group=path_graph_path_service_client_cb_group,
                qos_profile=qos_profile_services_default
            );
        else:
            return;
        
        
        
        self.__notify_path_publisher: Publisher = None;
        if self.__notify_path_publisher == None:
            notify_path_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__notify_path_publisher = self.__node.create_publisher(
                topic=NOTIFY_PATH_TOPIC,
                msg_type=route.Path,
                qos_profile=qos_profile_system_default,
                callback_group=notify_path_publisher_cb_group  
            );
        else:
            return;
        
    def convert_path_request(self, path_request: Path.Request) -> Path.Response | None:
        self.__log.info(f"{PATH_GRAPH_PATH_SERVICE_NAME} Path request\n{ros_message_dumps(message=path_request)}");
        
        is_path_graph_path_service_server_ready: bool = self.__path_graph_path_service_client.wait_for_service(timeout_sec=1.0);

        if is_path_graph_path_service_server_ready:
            path_response: Path.Response = self.__path_graph_path_service_client.call(request=path_request);

            if path_response is None:
                self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} Path Response is None...");
                self.__error_service.error_report_publish(error_code="999");
                return None;
            else:
                path: route.Path = path_response.path;
                self.__notify_path_publisher.publish(msg=path);
                return path_response;
        else:
            self.__log.error(f"{PATH_GRAPH_PATH_SERVICE_NAME} server is not ready");
            self.__error_service.error_report_publish(error_code="999");
        

__all__: list[str] = ["PathService"];