#include "manager/request/mission/mission_manager.hxx"

ktp::data::MissionManager::MissionManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_from_itf_subscription_opts;
    mission_from_itf_subscription_opts.callback_group = this->mission_from_itf_subscription_cb_group_;
    this->mission_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Mission>(
        MISSION_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::MissionManager::mission_from_itf_subscription_cb, this, _1),
        mission_from_itf_subscription_opts);
}

ktp::data::MissionManager::~MissionManager()
{
}

void ktp::data::MissionManager::mission_from_itf_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb)
{
}