from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import ControlReport;


NOTIFY_CONTROL_REPORT_FROM_TASK_CTRL_TOPIC: str = "/rms/ktp/task/notify/control/report";


class ControlReportService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        self.__control_report: ControlReport = None;
        
        notify_control_report_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__notify_control_report_subscription: Subscription = self.__node.create_subscription(
            topic=NOTIFY_CONTROL_REPORT_FROM_TASK_CTRL_TOPIC,
            msg_type=ControlReport,
            qos_profile=qos_profile_system_default,
            callback_group=notify_control_report_subscription_cb_group,
            callback=self.notify_control_report_subscription
        );
        
    @property
    def control_report(self) -> ControlReport:
        return self.__control_report;
    
    @control_report.setter
    def control_report(self, control_report: ControlReport) -> None:
        self.__control_report = control_report;
        
    def notify_control_report_subscription(self, control_report_cb: ControlReport) -> None:
        self.__control_report = control_report_cb;
    
    
__all__: list[str] = ["ControlReportService"];