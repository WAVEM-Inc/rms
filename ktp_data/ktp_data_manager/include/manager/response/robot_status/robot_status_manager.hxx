#ifndef ROBOT_STATUS_MANAGER__HXX
#define ROBOT_STATUS_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/status.hpp>
#include <ktp_data_msgs/msg/status_service.hpp>
#include <ktp_data_msgs/msg/status_service_env.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>

#include <robot_status_msgs/msg/velocity_status.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "model/model_enums.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10
#define DEFAULT_DOUBLE 0.0

#define STATUS_TO_ITF_TOPIC "/rms/ktp/data/robot_status"
#define STATUS_TO_ITF_RATE 1000
#define ROBOT_NAVIGATION_STATUS_FROM_TASK_CTRL_TOPIC "/rms/ktp/task/navigation/status"

#define BATTERY_STATE_TOPIC "/sensor/battery/state"
#define VELOCITY_STATE_TOPIC "/drive/velocity/state"
#define GPS_TOPIC "/sensor/ublox/fix"
#define RTT_ODOM_TOPIC "/drive/rtt_odom"
#define TEMPERATURE_TOPIC "/sensor/temp/temperature"
#define HUMIDITY_TOPIC "/sensor/temp/humidity"

#define MAP_ID "966"
#define COORD_CORD_SLAM "SLAM"
#define COORD_CORD_GPS "WGS84"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class RobotStatusManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr robot_status_to_itf_publisher_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr robot_status_to_itf_publisher_timer_;
            void robot_status_publisher_timer_cb();

            rclcpp::CallbackGroup::SharedPtr robot_status_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Status>::SharedPtr robot_status_to_itf_publisher_;

            sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb_;
            bool battery_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr battery_state_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;
            void battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb);

            robot_status_msgs::msg::VelocityStatus::SharedPtr velocity_state_cb_;
            bool velocity_state_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr velocity_state_subscription_cb_group_;
            rclcpp::Subscription<robot_status_msgs::msg::VelocityStatus>::SharedPtr velocity_state_subscription_;
            void velocity_state_subscription_cb(const robot_status_msgs::msg::VelocityStatus::SharedPtr velocity_state_cb);

            sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_;
            bool gps_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
            void gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix_cb);

            geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb_;
            bool rtt_odom_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr rtt_odom_subscription_cb_group_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rtt_odom_subscription_;
            void rtt_odom_subscription_cb(const geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb);
            
            sensor_msgs::msg::Temperature::SharedPtr temperature_cb_;
            bool temperature_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr temperature_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_subscription_;
            void temperature_subscription_cb(const sensor_msgs::msg::Temperature::SharedPtr temperature_cb);

            sensor_msgs::msg::RelativeHumidity::SharedPtr humidity_cb_;
            bool humidity_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr humidity_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_subscription_;
            void humidity_subscription_cb(const sensor_msgs::msg::RelativeHumidity::SharedPtr humidity_cb);

            ktp_data_msgs::msg::Status::SharedPtr robot_navigation_status_cb_;
            bool robot_navigation_status_cb_flag_;
            rclcpp::CallbackGroup::SharedPtr robot_navigation_status_from_task_ctrl_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Status>::SharedPtr robot_navigation_status_from_task_ctrl_subscription_;
            void robot_navigation_status_from_task_ctrl_cb(const ktp_data_msgs::msg::Status::SharedPtr robot_navigation_cb);

            ktp_data_msgs::msg::StatusService build_service();
            ktp_data_msgs::msg::Status build_robot_status();

        public:
            explicit RobotStatusManager(const rclcpp::Node::SharedPtr node);
            virtual ~RobotStatusManager();

        public:
            using SharedPtr = std::shared_ptr<RobotStatusManager>;
        };
    }
}

#endif // ROBOT_STATUS_MANAGER__HXX