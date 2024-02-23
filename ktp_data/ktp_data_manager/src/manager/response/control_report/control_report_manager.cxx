#include "manager/response/control_report/control_report_manager.hxx"

ktp::data::ControlReportManager::ControlReportManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->control_report_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions control_report_publisher_opts;
    control_report_publisher_opts.callback_group = this->control_report_to_itf_publisher_cb_group_;
    this->control_report_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ControlReport>(
        CONTROL_REPORT_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        control_report_publisher_opts);
}

ktp::data::ControlReportManager::~ControlReportManager()
{
}