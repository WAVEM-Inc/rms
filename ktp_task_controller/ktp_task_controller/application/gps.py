from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.publisher import Publisher;
from rclpy.subscription import Subscription;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from rclpy.qos import qos_profile_system_default;
from sensor_msgs.msg import NavSatFix;
from ktp_task_controller.domain.gps import set_gps;

UBLOX_FIX_TOPIC_NAME: str = "/sensor/ublox/fix";


class GpsService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();
        
        self.__ublox_fix_subscription: Subscription = None;
        if self.__ublox_fix_subscription == None:
            ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
            self.__ublox_fix_subscription = self.__node.create_subscription(
                topic=UBLOX_FIX_TOPIC_NAME,
                msg_type=NavSatFix,
                callback_group=ublox_fix_subscription_cb_group,
                callback=self.ublox_fix_subscription_cb,
                qos_profile=qos_profile_system_default
            );
        else:
            return;
        
    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        set_gps(gps=ublox_fix_cb);
    

__all__: list[str] = ["GpsService"];