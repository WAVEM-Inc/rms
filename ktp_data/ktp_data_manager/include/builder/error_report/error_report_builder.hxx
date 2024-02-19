#ifndef ERROR_REPORT_BUILDER__HXX
#define ERROR_REPORT_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/error_report.hpp>

#include "model/model_enums.hxx"

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
        };
    }
}

#endif