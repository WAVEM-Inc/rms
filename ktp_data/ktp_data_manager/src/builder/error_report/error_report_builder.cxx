#include "builder/error_report/error_report_builder.hxx"

ktp::build::ErrorReportBuilder::ErrorReportBuilder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
}

ktp::build::ErrorReportBuilder::~ErrorReportBuilder()
{
}