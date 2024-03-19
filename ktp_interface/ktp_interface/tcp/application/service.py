#-*- coding:utf-8 -*-

import json;
import socket;
import importlib;
import threading;

from ..presentation.IoTMakersDeviceClient import IoTMakersDeviceClient

from rclpy.node import Node

from typing import Any
from datetime import datetime;

from rosbridge_library.internal import message_conversion;

from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_msgs.msg import ServiceStatusTask;
from ktp_data_msgs.msg import Control;
from ktp_data_msgs.msg import ControlReport;
from ktp_data_msgs.msg import DetectedObject;

im_client: IoTMakersDeviceClient = IoTMakersDeviceClient();

thread_run_flag: bool = False;

IM_SERVER_ADDR: str = "14.63.249.103";
IM_SERVER_PORT: int = 32139;
IM_DEV_ID: str = "KECDSEMITB001";
IM_DEV_PW: str = "1234";
IM_DEV_GW: str = "M_OPENRM_UNMANNED_SOLUTION";
IM_LOGLEVEL: int = 3;  # 1:ERR, 2:INFO, 3:DEBUG


##########################################
#   리소스 설정(제어) 요청 처리 핸들러
##########################################
def OnResourceSetRequestHandler(pktBody, dev_id, resource_id, properties_in_jstr) -> int:
    print("OnResourceSetRequestHandler()-->", dev_id, resource_id, properties_in_jstr)

    # YOUR CONTROL CODE HERE
    properties = json.loads(properties_in_jstr);

    print(json.dumps(properties, indent=4));

    #IM_RESP_CODE_2004_Changed
    return 2004;


##########################################
#   특정 리소스(resource_id) 조회 요청 처리 핸들러
##########################################
def OnResourceRetrieveOneRequestHandler(pktBody, dev_id, resource_id) -> int:
    print("OnResourceSetRequestHandler()-->", dev_id, resource_id)

    # YOUR CONTROL CODE HERE
    properties = {
        "ectestString": "hello",
        "ectestDouble": 99.55,
        "ectestInteger": 100,
        "ectestBoolean": True
    };

    rc = im_client.ImResourceRetrieveSetResource(pktBody, resource_id, json.dumps(properties))
    if rc < 0:
        print("fail ImResourceRetrieveSetResource()");
        ## Internal Error
        return 5000;

    #IM_RESP_CODE_2000_OK
    return 2000;


##########################################
#   본 디바이스(dev_id)의 모든 리소스 조회 요청 처리 핸들러
##########################################
def OnResourceRetrieveAllRequestHandler(pktBody, dev_id) -> int:
    print("OnResourceSetRequestHandler()-->", dev_id)

    # YOUR CONTROL CODE HERE

    # 1. append FIRST RESOURCE
    resource_id = "testAllTypeUri";

    properties = {
        "ectestString": "hello",
        "ectestDouble": 99.55
    };

    rc = im_client.ImResourceRetrieveAppendResource(pktBody, resource_id, json.dumps(properties))
    if rc < 0:
        print("fail ImResourceRetrieveAppendResource()");
        ## Internal Error
        return 5000;

    # 2. append SECOND RESOURCE
    resource_id = "testAllTypeUri";

    properties = {
        "ectestInteger": 100,
        "ectestBoolean": True
    };

    rc = im_client.ImResourceRetrieveAppendResource(pktBody, resource_id, json.dumps(properties))
    if rc < 0:
        print("fail ImResourceRetrieveAppendResource()");
        ## Internal Error
        return 5000;

    #IM_RESP_CODE_2000_OK
    return 2000;


def tcp_send_resource(resource_id: str, properties: Any) -> int:
    properties_dumped: str = json.dumps(properties);
    print(f"Send Resource of id [{resource_id}], with property : [{properties_dumped}]");

    rc: int = 0;

    rc = im_client.ImResourceNotificationInit();
    if rc < 0:
        print("Failed ImResourceNotificationInit()");
        return -1;

    rc = im_client.ImResourceNotificationAppendResource(resource_id, properties_dumped);
    if rc < 0:
        print("Failed ImResourceNotificationAppendResource()");
        return -1;

    rc = im_client.ImResourceNotificationSend();
    if rc < 0:
        print("Failed ImResourceNotificationSend()");
        return -1;

    print(f"Succeeded Send Resource to id : [{resource_id}] with : [{properties_dumped}]");

    return 0;


##########################################
# 수신 처리용 쓰레드
##########################################
def polling_thread_cb(n) -> None:
    rc: int = 0;

    global thread_run_flag;

    while thread_run_flag == True:
        rc = im_client.ImPoll();
        if rc < 0:
            print("FAIL ImPoll()...");
            thread_run_flag = False;
            break;

        im_client.ImMSleep(100);

    print("_thread_poll_sample()..., DONE.");


def tcp_initialize() -> int:
    rc: int = 0;

    print("ImInit()....");
    rc = im_client.ImInit(IM_LOGLEVEL);
    if rc < 0:
        return -1;

    print("ImSetControlCallBackHandler()...");
    im_client.ImSetControlCallBackHandler(
        OnResourceSetRequestHandler=OnResourceSetRequestHandler,
        OnResourceRetrieveOneRequestHandler=OnResourceRetrieveOneRequestHandler,
        OnResourceRetrieveAllRequestHandler=OnResourceRetrieveAllRequestHandler
    );

    print(f"ImConnectTo()...[{IM_SERVER_ADDR}:{IM_SERVER_PORT}]");
    rc = im_client.ImConnectTo(IM_SERVER_ADDR, IM_SERVER_PORT);
    if rc < 0:
        im_client.ImRelease();
        return -1;

    print(f"ImAuthDevice()... {IM_DEV_ID}, {IM_DEV_PW}, {IM_DEV_GW}");
    rc = im_client.ImAuthDevice(IM_DEV_ID, IM_DEV_PW, IM_DEV_GW);
    if rc < 0:
        im_client.ImDisconnect();
        im_client.ImRelease();
        return -1;

    im_client.ImTurnCircuitBreakerOff();

    global thread_run_flag;
    thread_run_flag = True;

    return 0;


def tcp_release() -> None:
    rc: int = 0;

    rc = im_client.ImDisconnect();
    if rc < 0:
        print("Failed ImDisconnect()...");
        return;

    rc = im_client.ImRelease();
    if rc < 0:
        print("Failed ImRelease()...");
        return;
