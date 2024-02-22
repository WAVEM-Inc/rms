#ifndef RESPONSE_MANAGER__HXX
#define RESPONSE_MANAGER__HXX

#include "builder/builder.hxx"

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

#include <ktp_data_msgs/msg/graph_list.hpp>
#include <ktp_data_msgs/msg/graph.hpp>
#include <ktp_data_msgs/msg/graph_node_list.hpp>
#include <ktp_data_msgs/msg/graph_edge_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_vertices.hpp>

#define STATUS_TO_ITF_TOPIC "/ktp/data/robot_status"
#define STATUS_TO_ITF_RATE 1000

#define SERVICE_STATUS_TO_ITF_TOPIC "/ktp/data/service_status"
#define SERVICE_STATUS_FROM_TASK_CTRL_TOPIC "/ktp/task/service_status"

#define ERROR_REPORT_TO_ITF_TOPIC "/ktp/data/error_report"

#define CONTROL_REPORT_TO_ITF_TOPIC "/ktp/data/control_report"

#define GRAPH_LIST_TO_ITF_TOPIC "/ktp/data/graph_list"

namespace ktp
{
    namespace data
    {
        class ResponseManager
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::build::MainBuilder::SharedPtr builder_;
            
            rclcpp::CallbackGroup::SharedPtr robot_status_to_itf_publisher_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr robot_status_to_itf_publisher_timer_;
            void robot_status_publisher_timer_cb();

            rclcpp::CallbackGroup::SharedPtr robot_status_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Status>::SharedPtr robot_status_to_itf_publisher_;

            rclcpp::CallbackGroup::SharedPtr service_status_from_task_ctrl_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::ServiceStatus>::SharedPtr service_status_from_task_ctrl_subscription_;
            void service_status_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ServiceStatus::SharedPtr service_status_cb);

            rclcpp::CallbackGroup::SharedPtr service_status_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ServiceStatus>::SharedPtr service_status_to_itf_publisher_;

            rclcpp::CallbackGroup::SharedPtr error_report_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ErrorReport>::SharedPtr error_report_to_itf_publisher_;

            rclcpp::CallbackGroup::SharedPtr control_report_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ControlReport>::SharedPtr control_report_to_itf_publisher_;

            rclcpp::CallbackGroup::SharedPtr graph_list_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::GraphList>::SharedPtr graph_list_to_itf_publisher_;
        public:
            explicit ResponseManager(rclcpp::Node::SharedPtr node);
            virtual ~ResponseManager();
        public:
            using SharedPtr = std::shared_ptr<ResponseManager>;
        };
    }
}

#endif