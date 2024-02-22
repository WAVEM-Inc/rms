#ifndef ROBOT_STATUS_BUILDER__HXX
#define ROBOT_STATUS_BUILDER__HXX

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

#define BATTERY_STATE_TOPIC "/battery/state"
#define VELOCITY_STATE_TOPIC "/velocity/state"
#define GPS_TOPIC "/ublox/fix"
#define RTT_ODOM_TOPIC "/rtt_odom"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class RobotStatusBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;

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
            explicit RobotStatusBuilder(rclcpp::Node::SharedPtr node);
            virtual ~RobotStatusBuilder();
            ktp_data_msgs::msg::Status build_rbt_status();
        public:
            using SharedPtr = std::shared_ptr<RobotStatusBuilder>;
        };
    }
}

#endif