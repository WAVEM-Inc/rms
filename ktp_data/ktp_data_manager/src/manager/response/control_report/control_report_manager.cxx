#include "manager/response/control_report/control_report_manager.hxx"

ktp::data::ControlReportManager::ControlReportManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->control_report_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions control_report_response_publisher_opts;
    control_report_response_publisher_opts.callback_group = this->control_report_to_itf_publisher_cb_group_;
    this->control_report_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ControlReport>(
        NOTIFY_CONTROL_RESPONSE_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        control_report_response_publisher_opts);

    this->notify_mission_response_from_task_ctrl_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions notify_mission_response_from_task_ctrl_subscription_opts;
    notify_mission_response_from_task_ctrl_subscription_opts.callback_group = this->notify_mission_response_from_task_ctrl_subscription_cb_group_;
    this->notify_mission_response_from_task_ctrl_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::ControlReport>(
        NOTIFY_MISSION_RESPONSE_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ControlReportManager::notify_mission_response_from_task_ctrl_subscription_cb, this, _1),
        notify_mission_response_from_task_ctrl_subscription_opts);
}

ktp::data::ControlReportManager::~ControlReportManager()
{
}

void ktp::data::ControlReportManager::notify_mission_response_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ControlReport::SharedPtr notify_mission_cb)
{
    const ktp_data_msgs::msg::ControlReport &control_report = *(notify_mission_cb);

    this->control_report_to_itf_publisher_->publish(control_report);
}