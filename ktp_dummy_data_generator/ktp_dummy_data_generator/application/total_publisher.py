from datetime import datetime;
from rclpy.node import Node;
from rclpy.timer import Timer;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from sensor_msgs.msg import BatteryState;
from robot_status_msgs.msg import VelocityStatus;
from sensor_msgs.msg import NavSatFix;
from geometry_msgs.msg import PoseStamped;
from sensor_msgs.msg import Temperature;
from sensor_msgs.msg import RelativeHumidity;


BATTERY_STATE_TOPIC: str = "/sensor/battery/state";
VELOCITY_STATE_TOPIC: str = "/drive/velocity/state";
UBLOX_FIX_TOPIC: str = "/sensor/ublox/fix";
RTT_ODOM_TOPIC: str = "/drive/rtt_odom";
TEMPERATURE_TOPIC: str = "/sensor/temp/temperature";
HUMIDITY_TOPIC: str = "/sensor/temp/humidity";
DRIVE_ODOM_EULAR_TOPIC: str = "/drive/odom/eular";


class DummyTotalPublisher:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__count: float = 0.0;
        
        main_timer_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__main_timer: Timer = self.__node.create_timer(
            timer_period_sec=0.75,
            callback=self.main_timer_cb,
            callback_group=main_timer_cb_group
        );
        
        battery_state_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__battery_state_publisher: Publisher = self.__node.create_publisher(
            topic=BATTERY_STATE_TOPIC,
            msg_type=BatteryState,
            qos_profile=qos_profile_system_default,
            callback_group=battery_state_publisher_cb_group
        );
        
        velocity_state_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__velocity_state_publisher: Publisher = self.__node.create_publisher(
            topic=VELOCITY_STATE_TOPIC,
            msg_type=VelocityStatus,
            qos_profile=qos_profile_system_default,
            callback_group=velocity_state_publisher_cb_group
        );
        
        ublox_fix_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__ublox_fix_publisher: Publisher = self.__node.create_publisher(
            topic=UBLOX_FIX_TOPIC,
            msg_type=NavSatFix,
            qos_profile=qos_profile_system_default,
            callback_group=ublox_fix_publisher_cb_group
        );
        
        rtt_odom_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__rtt_odom_publisher: Publisher = self.__node.create_publisher(
            topic=RTT_ODOM_TOPIC,
            msg_type=PoseStamped,
            qos_profile=qos_profile_system_default,
            callback_group=rtt_odom_publisher_cb_group
        );
        
        temperature_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__temperature_publisher: Publisher = self.__node.create_publisher(
            topic=TEMPERATURE_TOPIC,
            msg_type=Temperature,
            qos_profile=qos_profile_system_default,
            callback_group=temperature_publisher_cb_group
        );
        
        humidity_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__humidity_publisher: Publisher = self.__node.create_publisher(
            topic=HUMIDITY_TOPIC,
            msg_type=RelativeHumidity,
            qos_profile=qos_profile_system_default,
            callback_group=humidity_publisher_cb_group
        );
        
        drive_odom_eular_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__drive_odom_eular_publisher: Publisher = self.__node.create_publisher(
            topic=DRIVE_ODOM_EULAR_TOPIC,
            msg_type=PoseStamped,
            qos_profile=qos_profile_system_default,
            callback_group=drive_odom_eular_publisher_cb_group
        );
        
    def main_timer_cb(self) -> None:
        self.battery_state_publish();
        self.velocity_state_publish();
        self.ublox_fix_publish();
        self.rtt_odom_publish();
        self.temperature_publish();
        self.humidity_publish();
        self.drive_odom_eular_publish();
    
    def battery_state_publish(self) -> None:
        battery_state: BatteryState = BatteryState();
        battery_state.voltage = 75.0;
        
        self.__battery_state_publisher.publish(msg=battery_state);
        
    def velocity_state_publish(self) -> None:
        velocity_state: VelocityStatus = VelocityStatus();
        velocity_state.current_velocity = 12.0;
        self.__count = self.__count + 1.0;
        velocity_state.distance = self.__count;
        
        if self.__count == 50.0:
            self.__count = 0.0;
        else:
            pass;
        
        self.__velocity_state_publisher.publish(msg=velocity_state);
    
    def ublox_fix_publish(self) -> None:
        nav_sat_fix: NavSatFix = NavSatFix();
        
        nav_sat_fix.longitude = 128.3690;
        nav_sat_fix.latitude = 36.11434;
        
        self.__ublox_fix_publisher.publish(msg=nav_sat_fix);
    
    def rtt_odom_publish(self) -> None:
        pose_stamped: PoseStamped = PoseStamped();
        pose_stamped.pose.orientation.y = 24.0;
        
        self.__rtt_odom_publisher.publish(msg=pose_stamped);
        
    def temperature_publish(self) -> None:
        temperature: Temperature = Temperature();
        temperature.temperature = 22.0;
        
        self.__temperature_publisher.publish(msg=temperature);

    def humidity_publish(self) -> None:
        humidity: RelativeHumidity = RelativeHumidity();
        humidity.relative_humidity = 12.0;
        
        self.__humidity_publisher.publish(msg=humidity);
        
    def drive_odom_eular_publish(self) -> None:
        pose_stamped: PoseStamped = PoseStamped();
        pose_stamped.pose.position.y = float(datetime.now().strftime("%M%S"));
        pose_stamped.pose.orientation.y = 320.0;
        
        self.__drive_odom_eular_publisher.publish(msg=pose_stamped);


__all__ = ["DummyTotalPublisher"];