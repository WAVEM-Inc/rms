#include "controller/notification/control/control_notificator.hxx"

ktp::controller::ControlNotificator::ControlNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->notificate_control_report_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notificate_control_publisher_opts;
    notificate_control_publisher_opts.callback_group = this->notificate_control_report_publisher_cb_group_;
    this->notificate_control_report_publihser_ = this->node_->create_publisher<ktp_data_msgs::msg::Control>(
            NOTIFICATE_CONTROL_TO_MGR_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
            notificate_control_publisher_opts);
}

ktp::controller::ControlNotificator::~ControlNotificator()
{
}