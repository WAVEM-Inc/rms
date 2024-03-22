import rclpy;

from rclpy.node import Node;

from ktp_interface.ros.application.response.rbt_status import RbtStatusManager;
from ktp_interface.ros.application.response.service_status import ServiceStatusManager;
from ktp_interface.ros.application.response.error_report import ErrorReportManager;
from ktp_interface.ros.application.response.control_report import ControlReportManager;
from ktp_interface.ros.application.response.graph_list import GraphListManager;
from ktp_interface.ros.application.response.obstacle_detect import ObstacleDetectManager;
from ktp_interface.ros.application.response.lidar_signal import LiDARSignalManager;


class ResponseManager:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;

        self.__rbt_status_manager: RbtStatusManager = RbtStatusManager(node=self.__node);
        self.__service_status_manager: ServiceStatusManager = ServiceStatusManager(node=self.__node);
        self.__error_report_manager: ErrorReportManager = ErrorReportManager(node=self.__node);
        self.__control_report_manager: ControlReportManager = ControlReportManager(node=self.__node);
        self.__graph_list_manager: GraphListManager = GraphListManager(node=self.__node);
        self.__obstacle_detect_manager: ObstacleDetectManager = ObstacleDetectManager(node=self.__node);
        self.__lidar_signal_manager: LiDARSignalManager = LiDARSignalManager(node=self.__node);


__all__ = ["ResponseManager"];
