#include "manager/request/object_detect/object_detect_manager.hxx"

ktp::data::ObjectDetectManager::ObjectDetectManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->object_detect_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions object_detect_from_itf_subscription_opts;
    object_detect_from_itf_subscription_opts.callback_group = this->object_detect_from_itf_subscription_cb_group_;
    this->object_detect_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::ObjectDetect>(
        OBJECT_DETECT_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ObjectDetectManager::object_detect_from_itf_subscription_cb, this, _1),
        object_detect_from_itf_subscription_opts);
}

ktp::data::ObjectDetectManager::~ObjectDetectManager()
{
}

void ktp::data::ObjectDetectManager::object_detect_from_itf_subscription_cb(const ktp_data_msgs::msg::ObjectDetect::SharedPtr object_detect_cb)
{
}