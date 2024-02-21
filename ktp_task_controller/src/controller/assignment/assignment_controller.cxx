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

ktp_data_msgs::msg::Mission ktp::controller::AssignmentController::transmiss_mission_to_notification()
{
    const ktp_data_msgs::msg::Mission &mission = this->mission_assigner_->transmiss_mission_to_notification()->get__mission();

    return mission;
}

ktp_data_msgs::msg::Control ktp::controller::AssignmentController::transmiss_control_to_notification()
{
    const ktp_data_msgs::msg::Control &control = this->control_assigner_->transmiss_control_to_notification()->get__control();

    return control;
}