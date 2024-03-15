#include "controller/notification/control/control_notificator.hxx"

ktp::controller::ControlNotificator::ControlNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->notify_control_report_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notify_control_publisher_opts;
    notify_control_publisher_opts.callback_group = this->notify_control_report_publisher_cb_group_;
    this->notify_control_report_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ControlReport>(
            NOTIFY_CONTROL_REPORT_TO_MGR_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
            notify_control_publisher_opts);
}

ktp::controller::ControlNotificator::~ControlNotificator()
{
}

void ktp::controller::ControlNotificator::notify_control_report(ktp_data_msgs::msg::Control control, int response_code)
{
    ktp_data_msgs::msg::ControlReport::UniquePtr control_report = std::make_unique<ktp_data_msgs::msg::ControlReport>();

    const std::string &create_time = get_current_time();
    control_report->set__create_time(create_time);

    const std::string &control_id = control.control_id;
    control_report->set__control_id(control_id);

    control_report->set__control_type("control");

    const std::string &control_code = control.control_code;
    control_report->set__control_code(control_code);

    control_report->set__response_code(response_code);

    const ktp_data_msgs::msg::ControlReport &&control_report_moved = std::move(*(control_report));

    this->notify_control_report_publisher_->publish(control_report_moved);
}