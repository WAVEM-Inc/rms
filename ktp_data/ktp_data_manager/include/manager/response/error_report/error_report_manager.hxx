#ifndef ERROR_REPORT_MANAGER__HXX
#define ERROR_REPORT_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/error_report.hpp>

#define DEFAULT_QOS 10

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