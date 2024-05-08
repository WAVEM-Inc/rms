from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.application.gps import GpsService;
from ktp_task_controller.application.obstacle import ObstacleService;
from ktp_task_controller.presentation.mission import MissionController;
from ktp_task_controller.presentation.control import ControlController;

class GateWay:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.declare_map_parameters();
        
        error_service: ErrorService = ErrorService(node=self.__node);
        gps_service: GpsService = GpsService(node=self.__node);
        obstacle_service: ObstacleService = ObstacleService(node=self.__node);
        mission_controller: MissionController = MissionController(node=self.__node);
        control_controller: ControlController = ControlController(node=self.__node);

    def declare_map_parameters(self) -> None:
        parameters_dict: dict = {
            "dev_id": "",
            "map_id": ""
        };

        for key, value in parameters_dict.items():
            self.__log.info(f"{self.__node.get_name()} Declaring key : [{key}], value : [{value}]");
            self.__node.declare_parameter(name=key, value=value);


__all__: list[str] = ["GateWay"];