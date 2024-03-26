#include "controller/assignment/control/control_assigner.hxx"

ktp::controller::ControlAssigner::ControlAssigner(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_assigner_ = std::make_shared<ktp::controller::MissionAssigner>(this->node_);

    this->mission_notificator_ = std::make_shared<ktp::controller::MissionNotificator>(this->node_);
    this->control_notificator_ = std::make_shared<ktp::controller::ControlNotificator>(this->node_);

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

    const std::string &control_code = control.control_code;

    if (control_code == CONTROL_MOVE_TO_DEST_CODE)
    {
        RCLCPP_INFO(this->node_->get_logger(), "MoveToDest");
        this->control_notificator_->notify_control_report(control, CONTROL_RECEPTION_SUCCEEDED_CODE);
        this->mission_assigner_->route_to_pose_send_goal();
    }
    else if (control_code == CONTROL_MS_COMPLETE_CODE)
    {
        RCLCPP_INFO(this->node_->get_logger(), "MsComplete");
        this->control_notificator_->notify_control_report(control, CONTROL_RECEPTION_SUCCEEDED_CODE);
        // this->mission_notificator_->notify_mission_status(MISSION_TASK_ENDED_CODE,this->mission_assigner_->return_current_mission_task());
    }
    else
    {
        return;
    }

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    response->set__result(true);
}