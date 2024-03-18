import threading;

from rclpy.node import Node;

from ktp_interface.tcp.application.service import TCP;
from ktp_interface.tcp.application.service import TCPService;
from ktp_interface.tcp.application.service import TCPTestService;

from ktp_interface.ros.manager.request.manager import RequestManager;
from ktp_interface.ros.manager.response.manager import ResponseManager;

NODE_NAME: str = "ktp_interface"


class KTPInterface(Node):

    def __init__(self) -> None:
        super().__init__(NODE_NAME);

        self.get_logger().info(f"{NODE_NAME} created");

        self.tcp_service: TCPService = TCPService(node=self);
        self.tcp_service.initialize();

        self.tcp_test_service: TCPTestService = TCPTestService(node=self);

        def run_tcp_thread() -> None:
            self.tcp_service.poll(1);
            print("_thread_notification_sample()...");

            while self.tcp_service.thread_run_flag:
                self.tcp_service.im_client.ImMSleep(1000);

        tcp_thread: threading.Thread = threading.Thread(target=run_tcp_thread);
        tcp_thread.start();
        # self.tcp_test_service.client_initialize();

        self.__request_manager: RequestManager = RequestManager(node=self, tcp_service=self.tcp_service);
        self.__response_manager: ResponseManager = ResponseManager(node=self, tcp_service=self.tcp_service);

    def stop_tcp_server(self) -> None:
        self.tcp_test_service.release();


__all__ = ["KTPInterface"];
