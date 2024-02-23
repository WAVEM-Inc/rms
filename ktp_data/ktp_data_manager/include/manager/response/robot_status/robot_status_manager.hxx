#ifndef ROBOT_STATUS_MANAGER__HXX
#define ROBOT_STATUS_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/status.hpp>
#include <ktp_data_msgs/msg/status_service.hpp>
#include <ktp_data_msgs/msg/status_service_env.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "model/model_enums.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10
#define DEFAULT_DOUBLE 0.0

#define STATUS_TO_ITF_TOPIC "/rms/ktp/data/robot_status"
#define STATUS_TO_ITF_RATE 1000

#define BATTERY_STATE_TOPIC "/sensor/battery/state"
#define VELOCITY_STATE_TOPIC "/drive/velocity/state"
#define GPS_TOPIC "/sensor/ublox/fix"
#define RTT_ODOM_TOPIC "/drive/rtt_odom"

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
            sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_;
            geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb_;

            bool battery_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr battery_state_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;
            void battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb);

            bool gps_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
            void gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix_cb);

            bool rtt_odom_cb_flag_ = false;
            rclcpp::CallbackGroup::SharedPtr rtt_odom_subscription_cb_group_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rtt_odom_subscription_;
            void rtt_odom_subscription_cb(const geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb);

            ktp_data_msgs::msg::StatusService build_service();

        public:
            explicit RobotStatusManager(rclcpp::Node::SharedPtr node);
            virtual ~RobotStatusManager();
            ktp_data_msgs::msg::Status build_robot_status();

        public:
            using SharedPtr = std::shared_ptr<RobotStatusManager>;
        };
    }
}

#endif // ROBOT_STATUS_MANAGER__HXX