#include "manager/request/request_manager.hxx"

ktp::data::RequestManager::RequestManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->builder_ = std::make_shared<ktp::build::MainBuilder>(this->node_);
    
    this->control_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions control_subscription_opts;
    control_subscription_opts.callback_group = this->control_subscription_cb_group_;
    this->control_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Control>(
        RBT_CONTROL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RequestManager::control_subscription_cb, this, _1),
        control_subscription_opts);

    this->mission_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_subscription_opts;
    mission_subscription_opts.callback_group = this->mission_subscription_cb_group_;
    this->mission_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Mission>(
        RBT_MISSION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RequestManager::mission_subscription_cb, this, _1),
        mission_subscription_opts);

    this->graph_list_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions graph_list_subscription_opts;
    graph_list_subscription_opts.callback_group = this->graph_list_subscription_cb_group_;
    this->graph_list_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::GraphList>(
        RBT_GRAPH_LIST_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RequestManager::graph_list_subscription_cb, this, _1),
        graph_list_subscription_opts);
}

ktp::data::RequestManager::~RequestManager()
{
}

void ktp::data::RequestManager::control_subscription_cb(const ktp_data_msgs::msg::Control::SharedPtr control_cb)
{

}

void ktp::data::RequestManager::mission_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb)
{

}

void ktp::data::RequestManager::graph_list_subscription_cb(const ktp_data_msgs::msg::GraphList::SharedPtr graph_list_cb)
{

}