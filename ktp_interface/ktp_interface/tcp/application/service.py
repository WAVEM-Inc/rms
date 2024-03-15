#-*- coding:utf-8 -*-

import socket
import json
from rclpy.node import Node

from typing import Any

from ..presentation.IoTMakersDeviceClient import IoTMakersDeviceClient


class TCP:
    pass;


class TCPService(TCP):

    def __init__(self, node: Node) -> None:
        super().__init__();
        self.im_client: IoTMakersDeviceClient = IoTMakersDeviceClient();
        self.__node: Node = node;
        pass;

    ##########################################
    # 	리소스 설정(제어) 요청 처리 핸들러
    ##########################################
    def on_request_cb_handler(self, pktBody, dev_id, resource_id, properties_in_jstr) -> int:
        self.__node.get_logger().info(
            f"OnResourceSetRequestHandler()--> {dev_id}, {resource_id}, {properties_in_jstr}");

        properties: Any = json.loads(properties_in_jstr);

        self.__node.get_logger().info("####################################################################");
        self.__node.get_logger().info(f"OnResourceSetRequestHandler()--> {json.dumps(properties, indent=4)}");
        self.__node.get_logger().info("####################################################################");

        return 2004;

    def on_retrieve_request_cb_handler(self, pktBody, dev_id, resource_id):
        pass;

    def on_retrieve_all_request_cb_handler(self, pktBody, dev_id):
        pass;

    ##########################################
    # 	리소스 값 전송
    ##########################################
    def send_resource(self, resource_id: str, properties: Any) -> int:
        properties = {
            "ectestString": "hello",
            "ectestDouble": 99.55,
            "ectestInteger": 100,
            "ectestBoolean": True
        };

        self.__node.get_logger().info(f"resource_id : {resource_id}, properties : {json.dumps(properties)}");

        rc = self.im_client.ImResourceNotificationInit()
        if rc < 0:
            self.__node.get_logger().info("fail ImResourceNotificationInit()");
            return -1;

        rc = self.im_client.ImResourceNotificationAppendResource(resource_id, json.dumps(properties));
        if rc < 0:
            self.__node.get_logger().info("fail ImResourceNotificationAppendResource()");
            return -1;

        rc = self.im_client.ImResourceNotificationSend();
        if rc < 0:
            self.__node.get_logger().info("fail ImResourceNotificationSend()");
            return -1;

        self.__node.get_logger().info(f"OK, SENT: {json.dumps(properties)}");

        return 0;

    def initialize(self) -> None:
        IM_SERVER_ADDR = "14.63.249.103";
        IM_SERVER_PORT = 32139;
        IM_DEV_ID = "KECDSEMITB001";
        IM_DEV_PW = "1234";
        IM_DEV_GW = "M_OPENRM_UNMANNED_SOLUTION";
        IM_LOGLEVEL = 3;  # 1:ERR, 2:INFO, 3:DEBUG

        self.__node.get_logger().info("ImInit()...");
        rc = self.im_client.ImInit(IM_LOGLEVEL);
        if rc < 0:
            return;

        self.__node.get_logger().info("ImSetControlCallBackHandler()...");
        self.im_client.ImSetControlCallBackHandler(self.on_request_cb_handler, self.on_retrieve_request_cb_handler,
                                                   self.on_retrieve_all_request_cb_handler);

        self.__node.get_logger().info(f"ImConnectTo()... {IM_SERVER_ADDR}, {IM_SERVER_PORT}");
        rc = self.im_client.ImConnectTo(IM_SERVER_ADDR, IM_SERVER_PORT);
        if rc < 0:
            self.im_client.ImRelease();
            return;
        else:
            print("######################################################################################");
            self.__node.get_logger().info(f"ImConnectTo() connected... {IM_SERVER_ADDR}, {IM_SERVER_PORT}");
            print("######################################################################################");

        self.__node.get_logger().info(f"ImAuthDevice()... {IM_DEV_ID}, {IM_DEV_PW}, {IM_DEV_GW}");
        rc = self.im_client.ImAuthDevice(IM_DEV_ID, IM_DEV_PW, IM_DEV_GW);
        if rc < 0:
            self.__node.get_logger().error("ImAuthDevice() failed...");
            self.im_client.ImDisconnect();
            self.im_client.ImRelease();
            return;
        else:
            print("######################################################################################");
            self.__node.get_logger().info(f"ImAuthDevice() succeeded... {IM_DEV_ID}, {IM_DEV_PW}, {IM_DEV_GW}");
            print("######################################################################################");

        # self.__node.get_logger().info("ImDisconnect()...");
        # rc = self.im_client.ImDisconnect();
        # self.im_client.ImMSleep(1000);


TEST_TCP_SOCKET_HOST: str = "192.168.0.187";
TEST_TCP_SOCKET_PORT: int = 12345;


class TCPTestService(TCP):

    def __init__(self, node: Node) -> None:
        super().__init__();
        self.__node: Node = node;
        self.__server_socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM);
        self.client_socket: socket.socket = None;
        self.client_address: Any = None;

    def initialize(self) -> None:
        self.__server_socket.bind((TEST_TCP_SOCKET_HOST, TEST_TCP_SOCKET_PORT));
        self.__server_socket.listen();
        self.__node.get_logger().info(f"TCP Test Server is waiting on [{TEST_TCP_SOCKET_HOST}:{TEST_TCP_SOCKET_PORT}]");

        self.client_socket, self.client_address = self.__server_socket.accept();
        self.__node.get_logger().info(f"TCP Client [{self.client_address}] is connected");

        while True:
            try:
                data: str = self.client_socket.recv(1024).decode("utf-8");
                self.__node.get_logger().info(f"TCP Request data : {data}");
                if not data:
                    break;

                parts: list[str] = data.split("&&");
                self.__node.get_logger().info(f"TCP Request parts : {parts}");
            except Exception as e:
                self.__node.get_logger().error(f"TCP Test Server error occurred : {e}");

    def release(self) -> None:
        self.__server_socket.close();
        self.__node.get_logger().info("TCP Server closed");


__all__ = ["TCPService"];
