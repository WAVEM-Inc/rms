#include "controller/controller.hxx"

ktp::controller::MainController::MainController()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] node has been created", NODE_NAME);
       
        this->assignment_controller_ = std::make_shared<ktp::controller::AssignmentController>(this->node_);
        this->notification_controller_ = std::make_shared<ktp::controller::NotificationController>(this->node_);
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(NODE_NAME, "failed to create %s node", NODE_NAME);
    }
}

ktp::controller::MainController::~MainController()
{

}