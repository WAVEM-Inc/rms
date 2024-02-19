#ifndef CONTORL_REPORT_BUILDER__HXX
#define CONTORL_REPORT_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/control_report.hpp>
#include <ktp_data_msgs/msg/control_report_data.hpp>
#include <ktp_data_msgs/msg/control_report_data_graph_list.hpp>

#include "model/model_enums.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class ControlReportBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp_data_msgs::msg::ControlReportDataGraphList build_data_graph_list();
            ktp_data_msgs::msg::ControlReportData build_data();
        public:
            explicit ControlReportBuilder(rclcpp::Node::SharedPtr node);
            virtual ~ControlReportBuilder();
            ktp_data_msgs::msg::ControlReport build_control_report();
        public:
            using SharedPtr = std::shared_ptr<ControlReportBuilder>;
        };
    }
}

#endif