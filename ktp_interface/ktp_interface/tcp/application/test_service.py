import json;
import socket;

from rclpy.node import Node;
from typing import Any;

TEST_TCP_SOCKET_HOST: str = "192.168.0.187";
TEST_TCP_SOCKET_PORT: int = 12345;

TEST_TCP_SOCKET_CLIENT_HOST: str = "192.168.0.54";
TEST_TCP_SOCKET_CLIENT_PORT: int = 12345;


class TCPTestService:

    def __init__(self, node: Node) -> None:
        super().__init__();
        self.__node: Node = node;
        self.__server_socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
        self.__client_socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
        self.__client_address: Any = None;

    def server_initialize(self) -> None:
        self.__server_socket.bind((TEST_TCP_SOCKET_HOST, TEST_TCP_SOCKET_PORT));
        self.__server_socket.listen();
        self.__node.get_logger().info(f"TCP Test Server is waiting on [{TEST_TCP_SOCKET_HOST}:{TEST_TCP_SOCKET_PORT}]");

        self.__client_socket, self.__client_address = self.__server_socket.accept();
        self.__node.get_logger().info(f"TCP Client [{self.__client_address}] is connected");

        while True:
            try:
                data: str = self.__client_socket.recv(1024).decode("utf-8");
                self.__node.get_logger().info(f"TCP Request data : {data}");
                if not data:
                    break;

                parts: list[str] = data.split("&&");
                self.__node.get_logger().info(f"TCP Request parts : {parts}");
            except Exception as e:
                self.__node.get_logger().error(f"TCP Test Server error occurred : {e}");

    def client_initialize(self) -> None:
        self.__node.get_logger().info(
            f"TCP Connecting to {(TEST_TCP_SOCKET_CLIENT_HOST, TEST_TCP_SOCKET_CLIENT_PORT)} server");
        self.__client_socket.connect((TEST_TCP_SOCKET_CLIENT_HOST, TEST_TCP_SOCKET_CLIENT_PORT));

    def send_resource(self, resource_id: str, resource: Any) -> None:
        self.__node.get_logger().info(f"TCP send_resource resource_id : {resource_id}, resource: {resource}");
        self.__client_socket.send(json.dumps(resource).encode());

    def release(self) -> None:
        self.__client_socket.close();
        self.__server_socket.close();
        self.__node.get_logger().info("TCP closed");


__all__ = ["TCPTestService"];
