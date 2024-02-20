#include "ktp_task_controller/ktp_task_controller.hxx"

ktp::task::Controller::Controller()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});

    
}