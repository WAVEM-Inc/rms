#include "controller/notification/mission/mission_notificator.hxx"

ktp::controller::MissionNotificator::MissionNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->assignment_controller_ = std::make_shared<ktp::controller::AssignmentController>(this->node_);

    this->notificate_mission_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notificate_mission_publihser_opts;
    notificate_mission_publihser_opts.callback_group = this->notificate_mission_publisher_cb_group_;
    this->notificate_mission_publihser_ = this->node_->create_publisher<ktp_data_msgs::msg::Mission>(
        NOTIFICATE_MISSION_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notificate_mission_publihser_opts);
}

ktp::controller::MissionNotificator::~MissionNotificator()
{
}

void ktp::controller::MissionNotificator::notificate_mission()
{
    const ktp_data_msgs::msg::Mission &mission = this->assignment_controller_->transmiss_mission_to_notification();

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "--------------------- Notificate Mission Publish -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "\n\trequest_time : [%s]\n\tmission_id : [%s]\n\towner : [%s]\n\tmission_code : [%s]",
        CSTR(mission.request_time),
        CSTR(mission.mission_id),
        CSTR(mission.owner),
        CSTR(mission.mission_code));
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    this->notificate_mission_publihser_->publish(mission);
}