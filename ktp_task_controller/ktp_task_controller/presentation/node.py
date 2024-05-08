from rclpy.node import Node;
from ktp_task_controller.presentation.gateway import GateWay;

NODE_NAME: str = "ktp_task_controller";


class KTPTaskController(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        gateway: GateWay = GateWay(node=self);


__all__: list[str] = ["KTPTaskController"];