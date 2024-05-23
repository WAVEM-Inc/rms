import os;
import yaml;
import json;
from typing import Any;
from ament_index_python.packages import get_package_share_directory;
from ktp_interface.tcp.libs.IoTMakersDeviceClient import IoTMakersDeviceClient;


im_client: IoTMakersDeviceClient = IoTMakersDeviceClient();
thread_run_flag: bool = False;
tcp_configuration_data: Any = {};


def set_tcp_connection_info() -> None:
    package_shared_directory: str = get_package_share_directory("ktp_interface");
    print(f"package_shared_directory : {package_shared_directory}");

    config_file_path: str = os.path.join(package_shared_directory, "config", "tcp_configuration.yaml");
    print(f"config_file_path : {config_file_path}");

    if not os.path.exists(config_file_path):
        raise FileNotFoundError(f"Configuration file '{config_file_path}' not found.");

    with open(config_file_path, "r") as f:
        global tcp_configuration_data;
        tcp_configuration_data = yaml.safe_load(f);

    print(f"tcp_configuration_data : {tcp_configuration_data}");


set_tcp_connection_info();
IM_SERVER_ADDR: str = tcp_configuration_data["server_address"];
IM_SERVER_PORT: int = tcp_configuration_data["server_port"];
IM_DEV_ID: str = tcp_configuration_data["dev_id"];
IM_DEV_PW: str = tcp_configuration_data["dev_pw"];
IM_DEV_GW: str = tcp_configuration_data["dev_gw"];
IM_LOGLEVEL: int = tcp_configuration_data["log_level"];  # 1:ERR, 2:INFO, 3:DEBUG

control: Any = {};
control_callback_flag: bool = False;

mission: Any = {};
mission_callback_flag: bool = False;

detected_object: Any = {};
detected_object_callback_flag: bool = False;


def get_control() -> Any:
    return control;


def set_control(_control: Any) -> None:
    global control;
    control = _control;


def get_control_callback_flag() -> bool:
    return control_callback_flag;


def set_control_callback_flag(_control_callback_flag: bool) -> None:
    global control_callback_flag;
    control_callback_flag = _control_callback_flag;


def get_mission() -> Any:
    return mission;


def set_mission(_mission: Any) -> None:
    global mission;
    mission = _mission;


def get_mission_callback_flag() -> bool:
    return mission_callback_flag;


def set_mission_callback_flag(_mission_callback_flag: bool) -> None:
    global mission_callback_flag;
    mission_callback_flag = _mission_callback_flag;


def get_detected_object() -> Any:
    return detected_object;


def set_detected_object(_detected_object: Any) -> None:
    global detected_object;
    detected_object = _detected_object;


def get_detected_object_flag() -> bool:
    return detected_object_callback_flag;


def set_detected_object_flag(_detected_object_flag: bool) -> None:
    global detected_object_callback_flag;
    detected_object_callback_flag = _detected_object_flag;


##########################################
#   리소스 설정(제어) 요청 처리 핸들러
##########################################
def on_resource_set_request_handler(pktBody, dev_id, resource_id, properties_in_jstr) -> int:
    print("OnResourceSetRequestHandler()-->", dev_id, resource_id, properties_in_jstr)

    properties: Any = json.loads(properties_in_jstr);
    print(json.dumps(properties, indent=4));

    if resource_id == "rbt_control":
        print("=============================== Control Request ===============================");
        set_control_callback_flag(True);
        set_control(properties);

        print(f"{resource_id} Callback : {json.dumps(get_control(), indent=4)}");
    elif resource_id == "rbt_mission":
        print("=============================== Mission Request ===============================");
        set_mission_callback_flag(True);
        set_mission(properties);

        print(f"{resource_id} Callback : {json.dumps(get_mission(), indent=4)}");
    elif resource_id == "rbt_detected_object":
        print("=============================== DetectedObject Request ===============================");
        set_detected_object_flag(True);
        set_detected_object(properties);

        print(f"{resource_id} Callback : {json.dumps(get_detected_object(), indent=4)}");

    #IM_RESP_CODE_2004_Changed
    return 2004;


##########################################
#   특정 리소스(resource_id) 조회 요청 처리 핸들러
##########################################
def on_resource_retrieve_one_request_handler(pktBody, dev_id, resource_id) -> int:
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
def on_resource_retrieve_all_request_handler(pktBody, dev_id) -> int:
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


def filter_empty_values(data: Any) -> Any:
    if isinstance(data, dict):
        return {
            key: filter_empty_values(value)
            for key, value in data.items()
            if value is not None and filter_empty_values(value) is not None
        }
    elif isinstance(data, list):
        return [
            filter_empty_values(item)
            for item in data
            if item is not None and filter_empty_values(item) is not None
        ]
    elif isinstance(data, str):
        return data if data.strip() != "" else None
    else:
        return data;


def tcp_send_resource(resource_id: str, properties: Any) -> int:
    properties_filtered: Any = filter_empty_values(properties);
    properties_filtered: str = json.dumps({k: v for k, v in properties_filtered.items() if v is not None});
    print(f"Send Resource of id [{resource_id}], with property : [{properties_filtered}]");

    rc: int = 0;

    rc = im_client.ImResourceNotificationInit();
    if rc < 0:
        print("Failed ImResourceNotificationInit()");
        return -1;

    rc = im_client.ImResourceNotificationAppendResource(resource_id, properties_filtered);
    if rc < 0:
        print("Failed ImResourceNotificationAppendResource()");
        return -1;

    rc = im_client.ImResourceNotificationSend();
    if rc < 0:
        print("Failed ImResourceNotificationSend()");
        return -1;

    print(f"Succeeded Send Resource to id : [{resource_id}] with : [{properties_filtered}]");

    return 0;


##########################################
# 수신 처리용 쓰레드
##########################################
def polling_thread_cb(n) -> None:
    rc: int = 0;

    global thread_run_flag;

    while thread_run_flag:
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
        OnResourceSetRequestHandler=on_resource_set_request_handler,
        OnResourceRetrieveOneRequestHandler=on_resource_retrieve_one_request_handler,
        OnResourceRetrieveAllRequestHandler=on_resource_retrieve_all_request_handler
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


__all__: list[str] = ["tcp_service"];