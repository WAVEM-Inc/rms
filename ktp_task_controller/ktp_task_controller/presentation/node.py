from rclpy.node import Node;
from ktp_task_controller.application.error import ErrorService;
from ktp_task_controller.application.gps import GpsService;
from ktp_task_controller.application.obstacle import ObstacleService;
from ktp_task_controller.presentation.mission import MissionController;
from ktp_task_controller.presentation.control import ControlController;

NODE_NAME: str = "ktp_task_controller";


class KTPTaskController(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        self.declare_common_parameters();
        
        ErrorService(node=self);
        GpsService(node=self);
        ObstacleService(node=self);
        MissionController(node=self);
        ControlController(node=self);

    def declare_common_parameters(self) -> None:
        parameters_dict: dict = {
            "dev_id": "",
            "map_id": ""
        };

        for key, value in parameters_dict.items():
            self.get_logger().info(f"{self.get_name()} Declaring key : [{key}], value : [{value}]");
            self.declare_parameter(name=key, value=value);


__all__: list[str] = ["KTPTaskController"];