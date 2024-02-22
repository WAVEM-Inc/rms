#ifndef SERVICE_STATUS_BUILDER__HXX
#define SERVICE_STATUS_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/service_status.hpp>
#include <ktp_data_msgs/msg/service_status_task.hpp>
#include <ktp_data_msgs/msg/service_status_task_data.hpp>

#include "model/model_enums.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10
#define DEFAULT_DOUBLE 0.0

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class ServiceStatusBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp_data_msgs::msg::ServiceStatusTaskData build_task_data();
            std::vector<ktp_data_msgs::msg::ServiceStatusTask> build_task();
        public:
            explicit ServiceStatusBuilder(rclcpp::Node::SharedPtr node);
            virtual ~ServiceStatusBuilder();
            ktp_data_msgs::msg::ServiceStatus build_rbt_service_status();
        public:
            using SharedPtr = std::shared_ptr<ServiceStatusBuilder>;
        };
    }
}

#endif