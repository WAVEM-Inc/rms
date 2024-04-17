#include "manager/request/request_manager.hxx"

ktp::data::RequestManager::RequestManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->control_manager_ = std::make_shared<ktp::data::ControlManager>(this->node_);
    this->mission_manager_ = std::make_shared<ktp::data::MissionManager>(this->node_);
    this->detected_object_manager_ = std::make_shared<ktp::data::DetectedObjectManager>(this->node_);

    RCLCPP_INFO(this->node_->get_logger(), "=============== Request Manager initialized ===============");
}

ktp::data::RequestManager::~RequestManager()
{
}