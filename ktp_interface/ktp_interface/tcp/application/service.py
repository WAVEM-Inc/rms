#-*- coding:utf-8 -*-

import json
from rclpy.node import Node

from typing import Any

from ..presentation.IoTMakersDeviceClient import IoTMakersDeviceClient


class TCPService():

    def __init__(self, node: Node) -> None:
        self.im_client: IoTMakersDeviceClient = IoTMakersDeviceClient();
        self.__node: Node = node;
        pass;
    
    ##########################################
    # 	리소스 설정(제어) 요청 처리 핸들러
    ##########################################
    def on_request_cb_handler(self, pktBody, dev_id, resource_id, properties_in_jstr) -> int:
        self.__node.get_logger().info(f"OnResourceSetRequestHandler()--> {dev_id}, {resource_id}, {properties_in_jstr}");

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
        self.__node.get_logger().info(json.dumps(properties));
        
        rc = self.im_client.ImResourceNotificationInit()
        if rc < 0 :
            self.__node.get_logger().info("fail ImResourceNotificationInit()");
            return -1;

        rc = self.im_client.ImResourceNotificationAppendResource(resource_id, json.dumps(properties));
        if rc < 0 :
            self.__node.get_logger().info("fail ImResourceNotificationAppendResource()");
            return -1;

        rc = self.im_client.ImResourceNotificationSend();
        if rc < 0 :
            self.__node.get_logger().info("fail ImResourceNotificationSend()");
            return -1;
        
        self.__node.get_logger().info(f"OK, SENT: {json.dumps(properties)}");

        return 0;
    
    
    def initialize(self) -> None:
        IM_SERVER_ADDR = "192.168.0.54";
        IM_SERVER_PORT = 23;
        IM_DEV_ID = "__testbyw111__";
        IM_DEV_PW = "12312";
        IM_DEV_GW = "KT_3RD_BYW";
        IM_LOGLEVEL = 3;    # 1:ERR, 2:INFO, 3:DEBUG

        self.__node.get_logger().info("ImInit()...");
        rc = self.im_client.ImInit(IM_LOGLEVEL);
        if rc < 0 :
            return;

        self.__node.get_logger().info("ImSetControlCallBackHandler()...");
        self.im_client.ImSetControlCallBackHandler(self.on_request_cb_handler, self.on_retrieve_request_cb_handler, self.on_retrieve_all_request_cb_handler);

        self.__node.get_logger().info(f"ImConnectTo()... {IM_SERVER_ADDR}, {IM_SERVER_PORT}");
        rc = self.im_client.ImConnectTo(IM_SERVER_ADDR, IM_SERVER_PORT);
        if rc < 0 :
            self.im_client.ImRelease();
            return;
        else :
            print("######################################################################################");
            self.__node.get_logger().info(f"ImConnectTo() connected... {IM_SERVER_ADDR}, {IM_SERVER_PORT}");
            print("######################################################################################");
        
        self.__node.get_logger().info(f"ImAuthDevice()... {IM_DEV_ID}, {IM_DEV_PW}, {IM_DEV_GW}");
        rc = self.im_client.ImAuthDevice(IM_DEV_ID, IM_DEV_PW, IM_DEV_GW);
        if rc < 0 :
            self.__node.get_logger().error("ImAuthDevice() failed...");
            self.im_client.ImDisconnect();
            self.im_client.ImRelease();
            return;
        else :
            print("######################################################################################");
            self.__node.get_logger().info(f"ImAuthDevice() succeeded... {IM_DEV_ID}, {IM_DEV_PW}, {IM_DEV_GW}");
            print("######################################################################################");
        
        # self.__node.get_logger().info("ImDisconnect()...");
        # rc = self.im_client.ImDisconnect();
        # self.im_client.ImMSleep(1000);


__all__ = ['ktp_interface.tcp.application.service'];