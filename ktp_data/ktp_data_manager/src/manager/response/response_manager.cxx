#include "manager/response/response_manager.hxx"

ktp::data::ResponseManager::ResponseManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->robot_status_manager_ = std::make_shared<ktp::data::RobotStatusManager>(this->node_);
    this->service_status_manager_ = std::make_shared<ktp::data::ServiceStatusManager>(this->node_);
    this->error_report_manager_ = std::make_shared<ktp::data::ErrorReportManager>(this->node_);
    this->control_report_manager_ = std::make_shared<ktp::data::ControlReportManager>(this->node_);
    this->graph_list_manager_ = std::make_shared<ktp::data::GraphListManager>(this->node_);
    this->obstacle_detect_manager_ = std::make_shared<ktp::data::ObstacleDetectManager>(this->node_);
    this->lidar_signal_manager_ = std::make_shared<ktp::data::LiDARSignalManager>(this->node_);
}

ktp::data::ResponseManager::~ResponseManager()
{
}