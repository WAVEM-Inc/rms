#include "controller/assignment/control/control_assigner.hxx"

ktp::controller::ControlAssigner::ControlAssigner(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->control_ = std::make_shared<ktp::domain::Control>();

    this->assign_control_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignControl>(
        ASSIGN_CONTROL_SERVICE_NAME,
        std::bind(&ktp::controller::ControlAssigner::assign_control_service_cb, this, _1, _2, _3));
}

ktp::controller::ControlAssigner::~ControlAssigner()
{
}

void ktp::controller::ControlAssigner::assign_control_service_cb(
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

ktp::domain::Control::SharedPtr ktp::controller::ControlAssigner::transmiss_control_to_notification()
{
    return this->control_;
}