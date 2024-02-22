#include "manager/request/request_manager.hxx"

ktp::data::RequestManager::RequestManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->builder_ = std::make_shared<ktp::build::MainBuilder>(this->node_);

    this->control_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions control_from_itf_subscription_opts;
    control_from_itf_subscription_opts.callback_group = this->control_from_itf_subscription_cb_group_;
    this->control_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Control>(
        CONTROL_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RequestManager::control_from_itf_subscription_cb, this, _1),
        control_from_itf_subscription_opts);

    this->mission_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_from_itf_subscription_opts;
    mission_from_itf_subscription_opts.callback_group = this->mission_from_itf_subscription_cb_group_;
    this->mission_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Mission>(
        MISSION_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RequestManager::mission_from_itf_subscription_cb, this, _1),
        mission_from_itf_subscription_opts);

    this->object_detect_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions object_detect_from_itf_subscription_opts;
    object_detect_from_itf_subscription_opts.callback_group = this->object_detect_from_itf_subscription_cb_group_;
    this->object_detect_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::ObjectDetect>(
        OBJECT_DETECT_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RequestManager::object_detect_from_itf_subscription_cb, this, _1),
        object_detect_from_itf_subscription_opts);
}

ktp::data::RequestManager::~RequestManager()
{
}

void ktp::data::RequestManager::control_from_itf_subscription_cb(const ktp_data_msgs::msg::Control::SharedPtr control_cb)
{
}

void ktp::data::RequestManager::mission_from_itf_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb)
{
}

void ktp::data::RequestManager::object_detect_from_itf_subscription_cb(const ktp_data_msgs::msg::ObjectDetect::SharedPtr object_detect_cb)
{
}