#ifndef SERVIC_STATUS_MANAGER__HXX
#define SERVIC_STATUS_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/service_status.hpp>
#include <ktp_data_msgs/msg/service_status_task.hpp>
#include <ktp_data_msgs/msg/service_status_task_data.hpp>

#define DEFAULT_QOS 10
#define DEFAULT_DOUBLE 0.0

#define SERVICE_STATUS_TO_ITF_TOPIC "/rms/ktp/data/service_status"
#define SERVICE_STATUS_FROM_TASK_CTRL_TOPIC "/rms/ktp/task/service_status"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class ServiceStatusManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr service_status_from_task_ctrl_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::ServiceStatus>::SharedPtr service_status_from_task_ctrl_subscription_;
            void service_status_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ServiceStatus::SharedPtr service_status_cb);

            rclcpp::CallbackGroup::SharedPtr service_status_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ServiceStatus>::SharedPtr service_status_to_itf_publisher_;
        public:
            explicit ServiceStatusManager(rclcpp::Node::SharedPtr node);
            virtual ~ServiceStatusManager();
        public:
            using SharedPtr = std::shared_ptr<ServiceStatusManager>;
        };
    }
}

#endif // SERVIC_STATUS_MANAGER__HXX