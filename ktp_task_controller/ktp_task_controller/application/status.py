from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.timer import Timer;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from ktp_data_msgs.msg import Status;
from ktp_data_msgs.msg import ServiceStatus;
from ktp_data_msgs.msg import ServiceStatusTask;
from ktp_data_msgs.msg import ServiceStatusTaskData;
from ktp_data_msgs.msg import MissionTask;
from ktp_data_msgs.msg import MissionTaskData;
from ktp_task_controller.utils import ros_message_dumps;
from ktp_task_controller.utils import get_current_time;
from ktp_task_controller.domain.mission import get_mission;
from ktp_task_controller.domain.status import get_driving_status;
from ktp_task_controller.domain.status import get_mission_total_distance;


NOTIFY_MISSION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/mission/status";
NOTIFY_NAVIGATION_STATUS_TOPIC_NAME: str = "/rms/ktp/task/notify/navigation/status";


class StatusService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        self.__map_id: str = self.__node.get_parameter("map_id").get_parameter_value().string_value;
        self.__navigation_status: Status = Status();
        
        self.__notify_mission_status_publisher = None;
        if self.__notify_mission_status_publisher is None:
            notify_mission_status_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__notify_mission_status_publisher = self.__node.create_publisher(
                topic=NOTIFY_MISSION_STATUS_TOPIC_NAME,
                msg_type=ServiceStatus,
                callback_group=notify_mission_status_publisher_cb_group,
                qos_profile=qos_profile_system_default
            );
        else:
            return;
        
        self.__notify_navigation_status_publisher: Publisher = None;
        if self.__notify_navigation_status_publisher is None:
            notify_navigation_status_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__notify_navigation_status_publisher = self.__node.create_publisher(
                topic=NOTIFY_NAVIGATION_STATUS_TOPIC_NAME,
                msg_type=Status,
                callback_group=notify_navigation_status_publisher_cb_group,
                qos_profile=qos_profile_system_default
            );
        else:
            return;
        
        self.__notify_status_timer: Timer = None;
        if self.__notify_status_timer is None:
            notify_status_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__notify_status_timer = self.__node.create_timer(
                timer_period_sec=0.8,
                callback_group=notify_status_timer_cb_group,
                callback=self.notify_mission_timer_cb
            );
        else:
            return;
        
    def notify_mission_status_publish(self, status: str) -> None:
        try:
            if get_mission() is not None:
                service_status: ServiceStatus = ServiceStatus();
                service_status.create_time = get_current_time();
                service_status.mission_code = get_mission().mission_code;
                service_status.mission_id = get_mission().mission_id;
                service_status.owner = get_mission().owner;

                mission_task: MissionTask = get_mission().task[0];
                service_status_task: ServiceStatusTask = ServiceStatusTask();
                service_status_task.task_id = mission_task.task_id;
                service_status_task.task_code = mission_task.task_code;
                service_status_task.status = status;
                service_status_task.seq = mission_task.seq;

                mission_task_data: MissionTaskData = mission_task.task_data;
                service_status_task_data: ServiceStatusTaskData = ServiceStatusTaskData();
                service_status_task_data.map_id = mission_task_data.map_id;
                service_status_task_data.source = mission_task_data.source;
                service_status_task_data.goal = mission_task_data.goal;
                service_status_task_data.lock_status = "0";
                service_status_task_data.distance = get_mission_total_distance();

                service_status.task = [service_status_task];
                service_status_task.task_data = service_status_task_data;

                self.__log.info(f"{NOTIFY_MISSION_STATUS_TOPIC_NAME} Service Status : {service_status_task.status}");
                self.__notify_mission_status_publisher.publish(msg=service_status);
            else:
                self.__log.info(f"{NOTIFY_MISSION_STATUS_TOPIC_NAME} Service Status Mission Is None");
                return;
        except AssertionError as ate:
            self.__log.error(f"{ate}");
            return;
    
    def notify_mission_timer_cb(self) -> None:
        self.notify_navigation_status_publish();
        
    def notify_navigation_status_publish(self) -> None:
        self.__navigation_status.map_id = self.__map_id;
        self.__navigation_status.drive_status = get_driving_status();

        if get_mission() != None:
            self.__navigation_status.from_node = get_mission().task[0].task_data.source;
            self.__navigation_status.to_node = get_mission().task[0].task_data.goal[0];
        else:
            self.__navigation_status.from_node = "";
            self.__navigation_status.to_node = "";

        self.__notify_navigation_status_publisher.publish(msg=self.__navigation_status);
    
    
__all__: list[str] = ["StatusService"];