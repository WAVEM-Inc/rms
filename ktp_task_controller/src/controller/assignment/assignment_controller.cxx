#include "controller/assignment/assignment_controller.hxx"

ktp::controller::AssignmentController::AssignmentController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_ = std::make_shared<ktp::domain::Mission>();
    this->control_ = std::make_shared<ktp::domain::Control>();

    this->assign_mission_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignMission>(
        ASSIGN_MISSION_SERVICE_NAME,
        std::bind(&ktp::controller::AssignmentController::assign_mission_service_cb, this, _1, _2, _3));

    this->assign_control_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignControl>(
        ASSIGN_CONTROL_SERVICE_NAME,
        std::bind(&ktp::controller::AssignmentController::assign_control_service_cb, this, _1, _2, _3));
}

ktp::controller::AssignmentController::~AssignmentController()
{
}

void ktp::controller::AssignmentController::assign_mission_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response)
{
    const ktp_data_msgs::msg::Mission &mission = request->mission;

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Assign Mission Service CB -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "\n\trequest_time : [%s]\n\tmission_id : [%s]\n\towner : [%s]\n\tmission_code : [%s]",
        CSTR(mission.request_time),
        CSTR(mission.mission_id),
        CSTR(mission.owner),
        CSTR(mission.mission_code));
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    this->mission_->set__mission(mission);

    response->set__result(true);
}

void ktp::controller::AssignmentController::assign_control_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Response> response)
{
    const ktp_data_msgs::msg::Control &control = request->control;

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Assign Control Service CB -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "\n\trequest_time : [%s]\n\tcontrol_id : [%s]\n\towner : [%s]\n\tcontrol_code : [%s]",
        CSTR(control.request_time),
        CSTR(control.control_id),
        CSTR(control.owner),
        CSTR(control.control_code));
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    this->control_->set__control(control);

    response->set__result(true);
}

ktp::domain::Mission::SharedPtr ktp::controller::AssignmentController::transmiss_mission_to_notification()
{
    return this->mission_;
}

ktp::domain::Control::SharedPtr ktp::controller::AssignmentController::transmiss_control_to_notification()
{
    return this->control_;
}