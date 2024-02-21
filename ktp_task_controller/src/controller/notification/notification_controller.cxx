#include "controller/notification/notification_controller.hxx"

ktp::controller::NotificationController::NotificationController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->notificate_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(NOTIFICATE_PUBLISHING_RATE),
        std::bind(&ktp::controller::NotificationController::notificate_timer_cb, this));
        
    this->mission_notificator_ = std::make_shared<ktp::controller::MissionNotificator>(this->node_);
    this->control_notificator_ = std::make_shared<ktp::controller::ControlNotificator>(this->node_);
}

ktp::controller::NotificationController::~NotificationController()
{
}

void ktp::controller::NotificationController::notificate_timer_cb()
{
    this->mission_notificator_->notificate_mission();
    this->control_notificator_->notificate_control();
}