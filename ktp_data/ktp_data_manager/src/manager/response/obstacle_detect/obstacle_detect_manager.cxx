#include "manager/response/obstacle_detect/obstacle_detect_manager.hxx"

ktp::data::ObstacleDetectManager::ObstacleDetectManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->obstacle_detect_from_task_ctrl_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions obstacle_detect_from_task_ctrl_subscription_opts;
    obstacle_detect_from_task_ctrl_subscription_opts.callback_group = this->obstacle_detect_from_task_ctrl_subscription_cb_group_;
    this->obstacle_detect_from_task_ctrl_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::ObstacleDetect>(
        OBSTACLE_DETECT_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ObstacleDetectManager::obstacle_detect_from_task_ctrl_subscription_cb, this, _1),
        obstacle_detect_from_task_ctrl_subscription_opts);

    this->obstacle_detect_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions obstacle_detect_to_itf_publisher_opts;
    obstacle_detect_to_itf_publisher_opts.callback_group = this->obstacle_detect_to_itf_publisher_cb_group_;
    this->obstacle_detect_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ObstacleDetect>(
        OBSTACLE_DETECT_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        obstacle_detect_to_itf_publisher_opts);

    RCLCPP_INFO(this->node_->get_logger(), "=============== Response ObstacleDetectManager initialized ===============");
}

ktp::data::ObstacleDetectManager::~ObstacleDetectManager()
{
}

void ktp::data::ObstacleDetectManager::obstacle_detect_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ObstacleDetect::SharedPtr obstacle_detect_cb)
{
    // RCLCPP_INFO(this->node_->get_logger(), "Obstacle Detect : {%s}", obstacle_detect_cb->create_time.c_str());
    this->obstacle_detect_to_itf_publisher_->publish(*(obstacle_detect_cb));
}