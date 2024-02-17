#ifndef BUILDER__HXX
#define BUILDER__HXX

#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/status.hpp>
#include <ktp_data_msgs/msg/status_service.hpp>
#include <ktp_data_msgs/msg/status_service_env.hpp>

#include <ktp_data_msgs/msg/service_status.hpp>
#include <ktp_data_msgs/msg/service_status_task.hpp>
#include <ktp_data_msgs/msg/service_status_task_data.hpp>

#include <ktp_data_msgs/msg/error_report.hpp>

#include <ktp_data_msgs/msg/control_report.hpp>
#include <ktp_data_msgs/msg/control_report_data.hpp>
#include <ktp_data_msgs/msg/control_report_data_graph_list.hpp>

#include <ktp_data_msgs/msg/control.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/graph_list.hpp>
#include <ktp_data_msgs/msg/graph.hpp>
#include <ktp_data_msgs/msg/graph_node_list.hpp>
#include <ktp_data_msgs/msg/graph_edge_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_vertices.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "model/model_enums.hxx"

#define DEFAULT_QOS 10

#define BATTERY_STATE_TOPIC "/battery/state"
#define VELOCITY_STATE_TOPIC "/velocity/state"
#define GPS_TOPIC "/ublox/fix"
#define RTT_ODOM_TOPIC "/rtt_odom"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class Builder final
        {
        private:
            rclcpp::Node::SharedPtr node_;
            sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb_;
            sensor_msgs::msg::NavSatFix::SharedPtr gps_cb_;
            geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb_;

            rclcpp::CallbackGroup::SharedPtr battery_state_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;
            void battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb);

            rclcpp::CallbackGroup::SharedPtr gps_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
            void gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_fix_cb);

            rclcpp::CallbackGroup::SharedPtr rtt_odom_subscription_cb_group_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rtt_odom_subscription_;
            void rtt_odom_subscription_cb(const geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb);

            std::string get_current_time();
        public:
            explicit Builder(rclcpp::Node::SharedPtr node);
            virtual ~Builder();
            ktp_data_msgs::msg::Status build_rbt_status();
            ktp_data_msgs::msg::ServiceStatus build_rbt_service_status();
        };
    }
}

#endif