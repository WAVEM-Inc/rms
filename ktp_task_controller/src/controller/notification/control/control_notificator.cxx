#include "controller/notification/control/control_notificator.hxx"

ktp::controller::ControlNotificator::ControlNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->assignment_controller_ = std::make_shared<ktp::controller::AssignmentController>(this->node_);

    this->notificate_control_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notificate_control_publihser_opts;
    notificate_control_publihser_opts.callback_group = this->notificate_control_publisher_cb_group_;
    this->notificate_control_publihser_ = this->node_->create_publisher<ktp_data_msgs::msg::Control>(
        NOTIFICATE_CONTROL_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notificate_control_publihser_opts);
}

ktp::controller::ControlNotificator::~ControlNotificator()
{
}

void ktp::controller::ControlNotificator::notificate_control()
{
    const ktp_data_msgs::msg::Control &control = this->assignment_controller_->transmiss_control_to_notification();

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "--------------------- Notificate Control Publish -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "\n\trequest_time : [%s]\n\tcontrol_id : [%s]\n\towner : [%s]\n\tcontrol_code : [%s]",
        CSTR(control.request_time),
        CSTR(control.control_id),
        CSTR(control.owner),
        CSTR(control.control_code));
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    this->notificate_control_publihser_->publish(control);
}