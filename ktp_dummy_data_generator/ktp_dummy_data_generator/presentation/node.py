from rclpy.node import Node;
from ktp_dummy_data_generator.application.processor import Processor;

NODE_NAME: str = "ktp_dummy_data_generator";


class KTPDummyDataGenerator(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");
        
        __processor: Processor = Processor(node=self);


__all__ = ["KTPDummyDataGenerator"];
