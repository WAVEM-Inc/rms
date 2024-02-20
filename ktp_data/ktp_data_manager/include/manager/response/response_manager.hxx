#ifndef RESPONSE_MANAGER__HXX
#define RESPONSE_MANAGER__HXX

#include "builder/builder.hxx"

#define STATUS_TO_ITF_TOPIC "/ktp/data/rbt_status"
#define STATUS_TO_ITF_RATE 1000

#define SERVICE_STATUS_TO_ITF_TOPIC "/ktp/task/rbt_service_status"
#define SERVICE_STATUS_FROM_TASK_CTRL_TOPIC "/ktp/data/rbt_service_status"

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
            rclcpp::Publisher<ktp_data_msgs::msg::ServiceStatus>::SharedPtr service_status_to_interface_publisher_;

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