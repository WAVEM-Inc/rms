from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_sensor_data;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from robot_status_msgs.msg import VelocityStatus;
from sensor_msgs.msg import BatteryState;
from sensor_msgs.msg import NavSatFix;
from sensor_msgs.msg import Temperature;
from sensor_msgs.msg import RelativeHumidity;
from geometry_msgs.msg import PoseStamped;
from ktp_data_msgs.msg import Status;


NAVIGATION_STATUS_TOPIC: str = "/rms/ktp/task/notify/navigation/status";
BATTERY_STATE_TOPIC: str = "/sensor/battery/state";
VELOCITY_STATE_TOPIC: str = "/drive/velocity/state";
UBLOX_FIX_TOPIC: str = "/sensor/ublox/fix";
RTT_ODOM_TOPIC: str = "/drive/rtt_odom";
TEMPERATURE_TOPIC: str = "/sensor/temp/temperature";
HUMIDITY_TOPIC: str = "/sensor/temp/humidity";


class RbtStatusService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();

        self.__rbt_status: Status = Status();
        
        battery_state_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__battery_state_subscription: Subscription = self.__node.create_subscription(
            topic=BATTERY_STATE_TOPIC,
            msg_type=BatteryState,
            qos_profile=qos_profile_sensor_data,
            callback_group=battery_state_subscription_cb_group,
            callback=self.battery_state_subscription_cb
        );
        
        velocity_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__velocity_status_subscription: Subscription = self.__node.create_subscription(
            topic=VELOCITY_STATE_TOPIC,
            msg_type=VelocityStatus,
            qos_profile=qos_profile_sensor_data,
            callback_group=velocity_status_subscription_cb_group,
            callback=self.velocity_status_subscription_cb
        );
        
        ublox_fix_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ublox_fix_subscription: Subscription = self.__node.create_subscription(
            topic=UBLOX_FIX_TOPIC,
            msg_type=NavSatFix,
            qos_profile=qos_profile_sensor_data,
            callback_group=ublox_fix_subscription_cb_group,
            callback=self.ublox_fix_subscription_cb
        );
        
        rtt_odom_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rtt_odom_subscription_cb_group: Subscription = self.__node.create_subscription(
            topic=RTT_ODOM_TOPIC,
            msg_type=PoseStamped,
            qos_profile=qos_profile_system_default,
            callback_group=rtt_odom_subscription_cb_group,
            callback=self.rtt_odom_subscription_cb
        );
        
        temperatre_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__temperature_subscription: Subscription = self.__node.create_subscription(
            topic=TEMPERATURE_TOPIC,
            msg_type=Temperature,
            qos_profile=qos_profile_sensor_data,
            callback_group=temperatre_subscription_cb_group,
            callback=self.temperature_subscription_cb
        );
        
        humidity_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__humidity_subscription: Subscription = self.__node.create_subscription(
            topic=HUMIDITY_TOPIC,
            msg_type=RelativeHumidity,
            qos_profile=qos_profile_sensor_data,
            callback_group=humidity_subscription_cb_group,
            callback=self.humidity_subscription_cb
        );
        
        navigation_status_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__navigation_status_subscription: Subscription = self.__node.create_subscription(
            topic=NAVIGATION_STATUS_TOPIC,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=navigation_status_subscription_cb_group,
            callback=self.navigation_status_subscription_cb
        );
        
    @property
    def rbt_status(self) -> Status:
        return self.__rbt_status;
    
    def battery_state_subscription_cb(self, battery_state_cb: BatteryState) -> None:
        battery_voltage: float = 0.0;
        if battery_voltage < 0 or battery_voltage > 100:
            battery_voltage = 0.0;
        else:
            battery_voltage = battery_state_cb.voltage;
            
        self.__rbt_status.battery = battery_state_cb.voltage;
        self.__rbt_status.charge = battery_state_cb.present;
    
    def velocity_status_subscription_cb(self, velocity_status_cb: VelocityStatus) -> None:
        self.__rbt_status.speed = abs(velocity_status_cb.current_velocity);
    
    def ublox_fix_subscription_cb(self, ublox_fix_cb: NavSatFix) -> None:
        self.__rbt_status.x = ublox_fix_cb.longitude;
        self.__rbt_status.y = ublox_fix_cb.latitude;
    
    def rtt_odom_subscription_cb(self, rtt_odom_cb: PoseStamped) -> None:
        self.__rbt_status.heading = rtt_odom_cb.pose.orientation.y;
    
    def temperature_subscription_cb(self, temperature_cb: Temperature) -> None:
        self.__rbt_status.service.env.temperature = temperature_cb.temperature;
    
    def humidity_subscription_cb(self, humidify_cb: RelativeHumidity) -> None:
        self.__rbt_status.service.env.humidity = humidify_cb.relative_humidity;
    
    def navigation_status_subscription_cb(self, navigation_status_cb: Status) -> None:
        self.__rbt_status.drive_status = navigation_status_cb.drive_status;
        self.__rbt_status.from_node = navigation_status_cb.from_node;
        self.__rbt_status.to_node = navigation_status_cb.to_node;
        
    
__all__: list[str] = ["RbtStatusService"];