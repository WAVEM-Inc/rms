from rclpy.node import Node;
from ktp_data_manager.presentation.rbt_status import RbtStatusController;
from ktp_data_manager.presentation.service_status import ServiceStatusController;
from ktp_data_manager.presentation.error_report import ErrorReportController;
from ktp_data_manager.presentation.control_report import ControlReportController;
from ktp_data_manager.presentation.control import ControlController;
from ktp_data_manager.presentation.mission import MissionController;
from ktp_data_manager.presentation.graph_list import GraphListController;


NODE_NAME: str = "ktp_data_manager";


class KTPDataManager(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");
        self.declare_common_parameters();
        
        RbtStatusController(node=self);
        ServiceStatusController(node=self);
        ErrorReportController(node=self);
        ControlReportController(node=self);
        ControlController(node=self);
        MissionController(node=self);
        GraphListController(node=self);

    def declare_common_parameters(self) -> None:
        parameters_dict: dict = {
            "dev_id": "",
            "map_id": "",
            "firmware_version": ""
        };

        for key, value in parameters_dict.items():
            self.get_logger().info(f"{self.get_name()} Declaring key : [{key}], value : [{value}]");
            self.declare_parameter(name=key, value=value);


__all__: list[str] = ["KTPDataManager"];