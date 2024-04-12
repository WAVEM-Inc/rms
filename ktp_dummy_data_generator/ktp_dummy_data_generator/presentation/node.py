from rclpy.node import Node;
from ktp_dummy_data_generator.application.route_to_pose import DummyRouteToPose;
from ktp_dummy_data_generator.application.total_publisher import DummyTotalPublisher;

NODE_NAME: str = "ktp_dummy_data_generator";


class KTPDummyDataGenerator(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");
        
        total_publisher: DummyTotalPublisher = DummyTotalPublisher(node=self);
        route_to_pose: DummyRouteToPose = DummyRouteToPose(node=self);


__all__ = ["KTPDummyDataGenerator"];