#include "manager/manager.hxx"

ktp::data::Manager::Manager()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] node has been created", NODE_NAME);
        this->request_manager_ = std::make_shared<ktp::data::RequestManager>(this->node_);
        this->response_manager_ = std::make_shared<ktp::data::ResponseManager>(this->node_);
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(NODE_NAME, "failed to create %s node", NODE_NAME);
    }
}

ktp::data::Manager::~Manager()
{
}