#include "manager/response/graph_list/graph_list_manager.hxx"

ktp::data::GraphListManager::GraphListManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->path_graph_graph_service_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->path_graph_graph_service_client_ = this->node_->create_client<path_graph_msgs::srv::Graph>(
        PATH_GRAPH_GRAPH_SERVICE_NAME,
        rmw_qos_profile_services_default,
        this->path_graph_graph_service_client_cb_group_);

    this->graph_list_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions graph_list_publisher_opts;
    graph_list_publisher_opts.callback_group = this->graph_list_to_itf_publisher_cb_group_;
    this->graph_list_to_itf_publisher_ = this->node_->create_publisher<std_msgs::msg::String>(
        GRAPH_LIST_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        graph_list_publisher_opts);
}

ktp::data::GraphListManager::~GraphListManager()
{
}

void ktp::data::GraphListManager::path_graph_graph_request()
{
    path_graph_msgs::srv::Graph::Request::SharedPtr graph_request = std::make_shared<path_graph_msgs::srv::Graph::Request>();

    const std::string &send_id = DEV_ID + get_current_time();
    graph_request->set__send_id(send_id);

    const bool &is_path_graph_graph_service_server_ready = this->path_graph_graph_service_client_->wait_for_service(std::chrono::seconds(1));

    if (is_path_graph_graph_service_server_ready)
    {
        rclcpp::Client<path_graph_msgs::srv::Graph>::FutureAndRequestId future_and_request_id = this->path_graph_graph_service_client_->async_send_request(graph_request);

        const std::future_status &future_status = future_and_request_id.wait_for(std::chrono::milliseconds(750));

        if (future_status == std::future_status::ready)
        {
            RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------\n");
            RCLCPP_INFO(this->node_->get_logger(), "----------------------------- Path Graph Graph Response ----------------------------------\n");

            const path_graph_msgs::srv::Graph::Response::SharedPtr response = future_and_request_id.future.get();

            const std::string &graph_list = response->graph_list;
            RCLCPP_INFO(this->node_->get_logger(), "graph_list : [%s]", graph_list.c_str());

            this->graph_list_publish(graph_list);

            RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------\n");
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Path Graph Graph Request Future Failed...");
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Path Graph Graph Service Server Is Not Ready...");
        return;
    }
}

void ktp::data::GraphListManager::graph_list_publish(std::string response_graph_list)
{
    std_msgs::msg::String::UniquePtr graph_list = std::make_unique<std_msgs::msg::String>();
    graph_list->set__data(response_graph_list);

    const std_msgs::msg::String &graph_list_moved = std::move(*(graph_list));

    this->graph_list_to_itf_publisher_->publish(graph_list_moved);
}