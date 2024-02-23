#include "manager/request/control/control_manager.hxx"

ktp::data::ControlManager::ControlManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->control_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions control_from_itf_subscription_opts;
    control_from_itf_subscription_opts.callback_group = this->control_from_itf_subscription_cb_group_;
    this->control_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Control>(
        CONTROL_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ControlManager::control_from_itf_subscription_cb, this, _1),
        control_from_itf_subscription_opts);
}

ktp::data::ControlManager::~ControlManager()
{
}

void ktp::data::ControlManager::control_from_itf_subscription_cb(const ktp_data_msgs::msg::Control::SharedPtr control_cb)
{
}