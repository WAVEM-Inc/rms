#ifndef ERROR_REPORT_BUILDER__HXX
#define ERROR_REPORT_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/error_report.hpp>

#include "model/model_enums.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class ErrorReportBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit ErrorReportBuilder(rclcpp::Node::SharedPtr node);
            virtual ~ErrorReportBuilder();
            ktp_data_msgs::msg::ErrorReport build_error_report();
        public:
            using SharedPtr = std::shared_ptr<ErrorReportBuilder>;
        };
    }
}

#endif