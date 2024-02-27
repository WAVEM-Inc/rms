#include "controller/notification/notification_controller.hxx"

ktp::controller::NotificationController::NotificationController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_notificator_ = std::make_shared<ktp::controller::MissionNotificator>(this->node_);
    this->control_notificator_ = std::make_shared<ktp::controller::ControlNotificator>(this->node_);
}

ktp::controller::NotificationController::~NotificationController()
{
}