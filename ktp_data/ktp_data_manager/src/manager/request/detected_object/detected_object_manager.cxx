#include "manager/request/detected_object/detected_object_manager.hxx"

ktp::data::DetectedObjectManager::DetectedObjectManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->detected_object_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions detected_object_from_itf_subscription_opts;
    detected_object_from_itf_subscription_opts.callback_group = this->detected_object_from_itf_subscription_cb_group_;
    this->detected_object_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::DetectedObject>(
        DETECTED_OBJECT_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::DetectedObjectManager::detected_object_from_itf_subscription_cb, this, _1),
        detected_object_from_itf_subscription_opts);

    this->object_detect_to_state_obstacle_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions object_detect_to_state_obstacle_publisher_opts;
    object_detect_to_state_obstacle_publisher_opts.callback_group = this->object_detect_to_state_obstacle_publisher_cb_group_;
    this->object_detect_to_state_obstacle_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::DetectedObject>(
        OBJECT_DETECT_TO_STATE_OBSTACLE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        object_detect_to_state_obstacle_publisher_opts);

    RCLCPP_INFO(this->node_->get_logger(), "=============== Request DetectedObjectManager initialized ===============");
}

ktp::data::DetectedObjectManager::~DetectedObjectManager()
{
}

void ktp::data::DetectedObjectManager::detected_object_from_itf_subscription_cb(const ktp_data_msgs::msg::DetectedObject::SharedPtr detected_object_cb)
{
    this->object_detect_to_state_obstacle_publisher_->publish(*(detected_object_cb));
}