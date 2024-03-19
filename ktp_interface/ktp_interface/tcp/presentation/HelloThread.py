#-*- coding:utf-8 -*-

import threading

import json

from .IoTMakersDeviceClient import IoTMakersDeviceClient

im_client = IoTMakersDeviceClient();

thread_run_flag = False


##########################################
#   리소스 설정(제어) 요청 처리 핸들러
##########################################
def OnResourceSetRequestHandler(pktBody, dev_id, resource_id, properties_in_jstr):
    print("OnResourceSetRequestHandler()-->", dev_id, resource_id, properties_in_jstr)

    # YOUR CONTROL CODE HERE
    properties = json.loads(properties_in_jstr);

    print(json.dumps(properties, indent=4));

    #IM_RESP_CODE_2004_Changed
    return 2004;


##########################################
#   특정 리소스(resource_id) 조회 요청 처리 핸들러
##########################################
def OnResourceRetrieveOneRequestHandler(pktBody, dev_id, resource_id):
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
def OnResourceRetrieveAllRequestHandler(pktBody, dev_id):
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


##########################################
#   리소스 값 전송 예제
##########################################
def example_resource_send_notification():
    resource_id = "testAllTypeUri";

    properties = {
        "ectestString": "hello",
        "ectestDouble": 99.55,
        "ectestInteger": 100,
        "ectestBoolean": True
    };

    print(json.dumps(properties));

    # 1. 전송 패킷 준비
    rc = im_client.ImResourceNotificationInit()
    if rc < 0:
        print("fail ImResourceNotificationInit()");
        return -1;

    # 2. 패킷에 리소스 추가; append FIRST RESOURCE
    rc = im_client.ImResourceNotificationAppendResource(resource_id, json.dumps(properties));
    if rc < 0:
        print("fail ImResourceNotificationAppendResource()");
        return -1;

    # 3. 패킷 전송
    rc = im_client.ImResourceNotificationSend();
    if rc < 0:
        print("fail ImResourceNotificationSend()");
        return -1;

    print("OK, SENT: ", json.dumps(properties));

    return 0;


##########################################
# 수신 처리용 쓰레드
##########################################
def _thread_poll_sample(n):
    rc = 0
    global thread_run_flag

    while thread_run_flag == True:
        rc = im_client.ImPoll();
        if rc < 0:
            print("FAIL ImPoll()...");
            thread_run_flag = False
            break;

        #print("_thread_poll_sample()...");
        im_client.ImMSleep(100);

    print("_thread_poll_sample()..., DONE.");


##########################################
# 리소스 Notofication 전송 쓰레드
##########################################
def _thread_notification_sample(n):
    rc = 0
    global thread_run_flag

    while thread_run_flag == True:
        # 리소스 전송 예제
        rc = example_resource_send_notification();
        if rc < 0:
            print("FAIL example_resource_send_notification()...");
            thread_run_flag = False
            break;

        print("_thread_notification_sample()...");
        im_client.ImMSleep(1000);


##########################################
# 시작 
##########################################
def iot_sample_run():
    # IM_SERVER_ADDR = "dev-gw-iotmakers.cloud.kt.com";
    # IM_SERVER_PORT = 30105;
    # IM_DEV_ID = "__testbyw111__";
    # IM_DEV_PW = "12312";
    # IM_DEV_GW = "KT_3RD_BYW";
    # IM_LOGLEVEL = 3;    # 1:ERR, 2:INFO, 3:DEBUG

    IM_SERVER_ADDR = "192.168.0.54";
    IM_SERVER_PORT = 23;
    IM_DEV_ID = "__testbyw111__";
    IM_DEV_PW = "12312";
    IM_DEV_GW = "KT_3RD_BYW";
    IM_LOGLEVEL = 3;  # 1:ERR, 2:INFO, 3:DEBUG

    print("ImInit()...");
    rc = im_client.ImInit(IM_LOGLEVEL);
    if rc < 0:
        return;

    print("ImSetControlCallBackHandler()...");
    im_client.ImSetControlCallBackHandler(OnResourceSetRequestHandler, OnResourceRetrieveOneRequestHandler,
                                         OnResourceRetrieveAllRequestHandler);

    print("ImConnectTo()...", IM_SERVER_ADDR, IM_SERVER_PORT);
    rc = im_client.ImConnectTo(IM_SERVER_ADDR, IM_SERVER_PORT);
    if rc < 0:
        im_client.ImRelease();
        return;

    print("ImAuthDevice()...", IM_DEV_ID, IM_DEV_PW, IM_DEV_GW);
    rc = im_client.ImAuthDevice(IM_DEV_ID, IM_DEV_PW, IM_DEV_GW);
    if rc < 0:
        im_client.ImDisconnect();
        im_client.ImRelease();
        return;

    #다중쓰레드 고속전송 옵션
    im_client.ImTurnCircuitBreakerOff();
    #im_client.ImTurnResponseWaitOff();

    global thread_run_flag
    thread_run_flag = True

    print("_thread_notification_sample()...");
    t_send = threading.Thread(target=_thread_notification_sample, args=(1,))
    t_send.start()

    print("_thread_poll_sample()...");
    t_poll = threading.Thread(target=_thread_poll_sample, args=(1,))
    t_poll.start()

    while thread_run_flag == True:
        #print("dummy()...");
        im_client.ImMSleep(1000);

    print("_thread_notification_sample()...Join");
    t_send.join()

    print("_thread_poll_sample()...Join");
    t_poll.join()

    print("ImDisconnect()...");
    rc = im_client.ImDisconnect();

    print("ImRelease()...");
    rc = im_client.ImRelease();


if __name__ == '__main__':
    while True:
        iot_sample_run()
        im_client.ImMSleep(1000);
