#include "manager/response/graph_list/graph_list_manager.hxx"

ktp::data::GraphListManager::GraphListManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->graph_list_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions graph_list_publisher_opts;
    graph_list_publisher_opts.callback_group = this->graph_list_to_itf_publisher_cb_group_;
    this->graph_list_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::GraphList>(
        GRAPH_LIST_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        graph_list_publisher_opts);
}

ktp::data::GraphListManager::~GraphListManager()
{
}