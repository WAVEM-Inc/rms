from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from std_msgs.msg import String;

ERROR_REPORT_TOPIC_NAME: str = "/rms/ktp/data/notify/error/status";


class ErrorService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
    
        self.__error_report_publisher: Publisher = None;
        if self.__error_report_publisher == None:
            error_report_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__error_report_publisher = self.__node.create_publisher(
                topic=ERROR_REPORT_TOPIC_NAME,
                msg_type=String,
                callback_group=error_report_publisher_cb_group,
                qos_profile=qos_profile_system_default
            );
        else:
            return;
        
    def error_report_publish(self, error_code: str) -> None:
        std_string: String = String();
        std_string.data = error_code;
        self.__error_report_publisher.publish(msg=std_string);
    

__all__: list[str] = ["ErrorService"];