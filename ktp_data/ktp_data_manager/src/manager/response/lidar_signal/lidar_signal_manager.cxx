#include "manager/response/lidar_signal/lidar_signal_manager.hxx"

ktp::data::LiDARSignalManager::LiDARSignalManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->obstacle_cooperative_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions obstacle_cooperative_subscription_opts;
    obstacle_cooperative_subscription_opts.callback_group = this->obstacle_cooperative_subscription_cb_group_;
    this->obstacle_cooperative_subscription_ = this->node_->create_subscription<std_msgs::msg::String>(
        OBSTACLE_COOPERATIVE_FROM_STATE_OBSTACLE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::LiDARSignalManager::obstacle_cooperative_subscription_cb, this, _1),
        obstacle_cooperative_subscription_opts);

    this->lidar_signal_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions lidar_signal_to_itf_publisher_opts;
    lidar_signal_to_itf_publisher_opts.callback_group = this->lidar_signal_to_itf_publisher_cb_group_;
    this->lidar_signal_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::LiDARSignal>(
        LIDAR_SIGNAL_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        lidar_signal_to_itf_publisher_opts);

    RCLCPP_INFO(this->node_->get_logger(), "=============== Response LiDARSignalManager initialized ===============");
}

ktp::data::LiDARSignalManager::~LiDARSignalManager()
{
}

void ktp::data::LiDARSignalManager::obstacle_cooperative_subscription_cb(const std_msgs::msg::String::SharedPtr obstacle_cooperative_cb)
{
    const std::string &obstacle_cooperative_flag = obstacle_cooperative_cb->data;

    RCLCPP_INFO(this->node_->get_logger(), "LiDAR Signal obstacle_cooperative_flag : [%s]", obstacle_cooperative_flag.c_str());
    this->lidar_signal_to_itf_publish(obstacle_cooperative_flag);
}

void ktp::data::LiDARSignalManager::lidar_signal_to_itf_publish(std::string obstacle_cooperative_flag)
{
    ktp_data_msgs::msg::LiDARSignal::UniquePtr lidar_signal = std::make_unique<ktp_data_msgs::msg::LiDARSignal>();
    const std::string &create_time = get_current_time();
    lidar_signal->set__create_time(create_time);
    lidar_signal->set__signal_type(obstacle_cooperative_flag);

    const ktp_data_msgs::msg::LiDARSignal &&lidar_signal_moved = std::move(*(lidar_signal));
    this->lidar_signal_to_itf_publisher_->publish(lidar_signal_moved);
}