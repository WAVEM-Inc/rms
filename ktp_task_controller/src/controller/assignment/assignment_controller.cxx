#include "controller/assignment/assignment_controller.hxx"

ktp::controller::AssignmentController::AssignmentController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_assigner_ = std::make_shared<ktp::controller::MissionAssigner>(this->node_);
    this->control_assigner_ = std::make_shared<ktp::controller::ControlAssigner>(this->node_);
}

ktp::controller::AssignmentController::~AssignmentController()
{
}