#ifndef ERROR_REPORT_MANAGER__HXX
#define ERROR_REPORT_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <ktp_data_msgs/msg/error_report.hpp>

#include "utils/utils.hxx"

#define DEFAULT_QOS 1

#define ERROR_RECEPTION_TOPIC "/rms/ktp/data/notify/error/status"
#define ERROR_REPORT_TO_ITF_TOPIC "/rms/ktp/data/error_report"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class ErrorReportManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr error_reception_subscription_cb_group_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_reception_subscription_;
            void error_reception_subscription_cb(const std_msgs::msg::String::SharedPtr error_reception_cb);

            rclcpp::CallbackGroup::SharedPtr error_report_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ErrorReport>::SharedPtr error_report_to_itf_publisher_;
        public:
            explicit ErrorReportManager(rclcpp::Node::SharedPtr node);
            virtual ~ErrorReportManager();
        public:
            using SharedPtr = std::shared_ptr<ErrorReportManager>;
        };
    }
}

#endif // ERROR_REPORT_MANAGER__HXX