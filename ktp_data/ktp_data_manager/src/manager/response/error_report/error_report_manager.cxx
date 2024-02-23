#include "manager/response/error_report/error_report_manager.hxx"

ktp::data::ErrorReportManager::ErrorReportManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->error_report_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions error_report_publisher_opts;
    error_report_publisher_opts.callback_group = this->error_report_to_itf_publisher_cb_group_;
    this->error_report_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ErrorReport>(
        ERROR_REPORT_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        error_report_publisher_opts);
}

ktp::data::ErrorReportManager::~ErrorReportManager()
{
}