from rclpy.node import Node;
from ktp_task_controller.application.processor import Processor;

NODE_NAME: str = "ktp_task_controller";


class KTPTaskController(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        self.__processor: Processor = Processor(node=self);


__all__ = ["KTPTaskController"];