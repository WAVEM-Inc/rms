NODE_NAME: str = "ktp_task_controller";
TASK_MANAGER_NODE_NAME: str = "ktp_task_manager";

ASSIGN_MISSION_SERVICE_NAME: str = f"/{NODE_NAME}/assign/mission";
PATH_GRAPH_PATH_SERVICE_NAME: str = "/path_graph_msgs/path";
PATH_GRAPH_GRAPH_SERVICE_NAME: str = "/path_graph_msgs/graph";
UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";
ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
NOTIFY_MISSION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/mission/status";
NOTIFY_NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/navigation/status";
ERROR_REPORT_TOPIC_NAME: str = "/rms/ktp/data/notify/error/status";
ASSIGN_CONTROL_SERVICE_NAME: str = f"/{NODE_NAME}/assign/control";
NOTIFY_CONTROL_REPORT_TOPIC_NAME: str = "/rms/ktp/task/notify/control/report";
NOTIFY_OBSTACLE_DETECT_TOPIC_NAME: str = "/rms/ktp/task/notify/obstacle_detect";
GRAPH_LIST_TOPIC: str = "/rms/ktp/task/notify/graph_list";
DRIVE_OBSTACLE_TOPIC: str = "/drive/obstacle/event";

CONVERT_TASK_TO_PATH_SERVICE_NAME: str = f"/{TASK_MANAGER_NODE_NAME}/path/convert";
LOOK_UP_PATH_SERVICE_NAME: str=  f"/{TASK_MANAGER_NODE_NAME}/path/look_up";
READY_TO_NAVIGATION_TOPIC_NAME: str = "/rms/ktp/task/navigation/ready";
NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/navigation/status";

CONTROL_CODE_STOP: str = "stop";
CONTROL_CODE_RELEASE: str = "release";
CONTROL_MS_CANCEL: str = "mscancel";
CONTROL_CODE_MOVE_TO_DEST: str = "movetodest";
CONTROL_CODE_MS_COMPLETE: str = "mscomplete";
CONTROL_CODE_GRAPH_SYNC: str = "graphsync";

DRIVE_STATUS_WAIT: int = 0;
DRIVE_STATUS_ON_DRIVE: int = 1;
DRIVE_STATUS_DRIVE_FINISHED: int = 2;
DRIVE_STATUS_CANCELLED: int = 3;
DRIVE_STATUS_OBJECT_DETECTED: int = 4;
DRIVE_STATUS_DRIVE_FAILED: int = 5;
DRIVE_STATUS_MISSION_IMPOSSIBLE: int = 14;

NAVIGATION_STATUS_READY_TO_MOVE: int = 21;