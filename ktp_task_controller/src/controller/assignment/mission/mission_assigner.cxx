#include "controller/assignment/mission/mission_assigner.hxx"

ktp::controller::MissionAssigner::MissionAssigner(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_ = std::make_shared<ktp::domain::Mission>();

    this->assign_mission_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignMission>(
        ASSIGN_MISSION_SERVICE_NAME,
        std::bind(&ktp::controller::MissionAssigner::assign_mission_service_cb, this, _1, _2, _3));
}

ktp::controller::MissionAssigner::~MissionAssigner()
{
}

void ktp::controller::MissionAssigner::assign_mission_service_cb(
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

ktp::domain::Mission::SharedPtr ktp::controller::MissionAssigner::transmiss_mission_to_notification()
{
    return this->mission_;
}