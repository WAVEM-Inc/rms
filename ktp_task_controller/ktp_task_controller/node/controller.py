from rclpy.node import Node;
from ktp_task_controller.application.mission_controller import MissionController;
from ktp_task_controller.application.control_controller import ControlController;

NODE_NAME: str = "ktp_task_controller";


class KTPTaskController(Node):

    def __init__(self) -> None:
        super().__init__(node_name=NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        self.__mission_controller: MissionController = MissionController(node=self);
        self.__control_controller: ControlController = ControlController(node=self);