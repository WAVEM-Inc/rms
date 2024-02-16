#ifndef BROADCASTER__HXX
#define BROADCASTER__HXX

#include "builder/builder.hxx"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rcutils/logging_macros.h>

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

#define NODE_NAME "ktp_data_manager"
#define DEFAULT_QOS 10

#define RBT_STATUS_TOPIC "/ktp_data/rbt_status"
#define RBT_SERVICE_STATUS_TOPIC "/ktp_data/rbt_service_status"
#define RBT_ERROR_REPORT_TOIPC "/ktp_data/error_report"
#define RBT_CONTROL_REPORT_TOPIC "/ktp_data/control_report"
#define RBT_CONTROL_TOPIC "/ktp_data/control"
#define RBT_MISSION_TOPIC "/ktp_data/mission"
#define RBT_GRAPH_LIST_TOPIC "/ktp_data/graph_list"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class BroadCaster final : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            
            std::shared_ptr<ktp::data::Builder> builder_;
            
            rclcpp::CallbackGroup::SharedPtr rbt_status_publisher_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr rbt_status_publisher_timer_;
            
            rclcpp::CallbackGroup::SharedPtr rbt_status_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Status>::SharedPtr rbt_status_publisher_;

            rclcpp::CallbackGroup::SharedPtr rbt_service_status_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ServiceStatus>::SharedPtr rbt_service_status_publisher_;

            rclcpp::CallbackGroup::SharedPtr rbt_error_report_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ErrorReport>::SharedPtr rbt_error_report_publisher_;

            rclcpp::CallbackGroup::SharedPtr rbt_control_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Control>::SharedPtr rbt_control_publisher_;

            rclcpp::CallbackGroup::SharedPtr rbt_control_report_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ControlReport>::SharedPtr rbt_control_report_publisher_;

            rclcpp::CallbackGroup::SharedPtr rbt_mission_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Mission>::SharedPtr rbt_mission_publisher_;

            rclcpp::CallbackGroup::SharedPtr rbt_graph_list_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::GraphList>::SharedPtr rbt_graph_list_publisher_;

            void rbt_status_publisher_timer_cb();
        public:
            explicit BroadCaster();
            virtual ~BroadCaster();
        };
    }
}

#endif