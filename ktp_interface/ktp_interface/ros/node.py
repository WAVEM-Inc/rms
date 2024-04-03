import threading;

from rclpy.node import Node;

from ktp_interface.ros.manager.request.manager import RequestManager;
from ktp_interface.ros.manager.response.manager import ResponseManager;

from ktp_interface.tcp.application.service import tcp_initialize;
from ktp_interface.tcp.application.service import polling_thread_cb;
from ktp_interface.tcp.application.service import thread_run_flag;

NODE_NAME: str = "ktp_interface";


class KTPInterface(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        tcp_rc: int = tcp_initialize();
        if tcp_rc < 0:
            self.get_logger().error("Failed to initialize tcp");

        self.__request_manager: RequestManager = RequestManager(node=self);
        self.__response_manager: ResponseManager = ResponseManager(node=self);

        print("Polling Thread...");
        tcp_polling_thread: threading.Thread = threading.Thread(target=polling_thread_cb, args=(1,));
        tcp_polling_thread.start();


__all__ = ["KTPInterface"];
