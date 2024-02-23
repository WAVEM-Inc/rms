#include "manager/request/request_manager.hxx"

ktp::data::RequestManager::RequestManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->control_manager_ = std::make_shared<ktp::data::ControlManager>(this->node_);
    this->mission_manager_ = std::make_shared<ktp::data::MissionManager>(this->node_);
    this->object_detect_manager_ = std::make_shared<ktp::data::ObjectDetectManager>(this->node_);
}

ktp::data::RequestManager::~RequestManager()
{
}