#include "manager/response/service_status/service_status_manager.hxx"

ktp::data::ServiceStatusManager::ServiceStatusManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_status_from_task_ctrl_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_status_from_task_ctrl_subscription_opts;
    mission_status_from_task_ctrl_subscription_opts.callback_group = this->mission_status_from_task_ctrl_subscription_cb_group_;
    this->mission_status_from_task_ctrl_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::ServiceStatus>(
        MISSION_STATUS_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ServiceStatusManager::mission_status_from_task_ctrl_subscription_cb, this, _1),
        mission_status_from_task_ctrl_subscription_opts);

    this->service_status_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions service_status_to_itf_publisher_opts;
    service_status_to_itf_publisher_opts.callback_group = this->service_status_to_itf_publisher_cb_group_;
    this->service_status_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ServiceStatus>(
        SERVICE_STATUS_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        service_status_to_itf_publisher_opts);
}

ktp::data::ServiceStatusManager::~ServiceStatusManager()
{
}

void ktp::data::ServiceStatusManager::mission_status_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ServiceStatus::SharedPtr mission_status_cb)
{
    RCLCPP_INFO(this->node_->get_logger(), "-------------------------------------------------------------------------------------------------\n");
    RCLCPP_INFO(this->node_->get_logger(), "--------------------------- Notify Mission(ServiceStatus) Callback ------------------------------\n");

    this->service_status_to_itf_publisher_->publish(*(mission_status_cb));
    RCLCPP_INFO(this->node_->get_logger(), "-------------------------------------------------------------------------------------------------\n");
}