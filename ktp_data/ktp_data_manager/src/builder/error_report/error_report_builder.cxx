#include "builder/error_report/error_report_builder.hxx"

ktp::build::ErrorReportBuilder::ErrorReportBuilder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
}

ktp::build::ErrorReportBuilder::~ErrorReportBuilder()
{
}

ktp_data_msgs::msg::ErrorReport ktp::build::ErrorReportBuilder::build_error_report()
{
    ktp_data_msgs::msg::ErrorReport::UniquePtr error_report = std::make_unique<ktp_data_msgs::msg::ErrorReport>();

    // ############################################
    // create_time
    // 생성 일시
    const std::string &create_time = get_current_time();
    error_report->set__create_time(create_time);
    // ############################################

    // ############################################
    // error_code
    // 장애 코드
    error_report->set__error_code("1004");
    // ############################################

    const ktp_data_msgs::msg::ErrorReport &&error_report_moved = std::move(*error_report);

    return error_report_moved;
}