from rclpy.node import Node;
from ktp_task_controller.application.processor import Processor;

NODE_NAME: str = "ktp_task_controller";


class KTPTaskController(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        self.declare_common_parameters();
        processor: Processor = Processor(node=self);
    
    def declare_common_parameters(self) -> None:
        parameters_dict: dict = {
            "device_id": "",
            "map_id": ""
        };

        for key, value in parameters_dict.items():
            self.get_logger().info(f"{self.get_name()} Declaring key : [{key}], value : [{value}]");
            self.declare_parameter(name=key, value=value);


__all__ = ["KTPTaskController"];