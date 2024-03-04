#include "controller/notification/mission/mission_notificator.hxx"

ktp::controller::MissionNotificator::MissionNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->notify_mission_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notify_mission_publisher_opts;
    notify_mission_publisher_opts.callback_group = this->notify_mission_publisher_cb_group_;
    this->notify_mission_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ControlReport>(
            NOTIFY_MISSION_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
            notify_mission_publisher_opts);
}

ktp::controller::MissionNotificator::~MissionNotificator()
{
}

void ktp::controller::MissionNotificator::notify_mission_status(ktp::domain::Mission::SharedPtr domain_mission)
{
    const int16_t &status_code = domain_mission->get__mission_status_code();
    const ktp_data_msgs::msg::Mission &mission = domain_mission->get__mission();

    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Notify Mission Status -----------------------");
    RCLCPP_INFO(this->node_->get_logger(), "\n\tstatus_code : [%hd]", status_code);

    ktp_data_msgs::msg::ControlReport::UniquePtr mission_report = std::make_unique<ktp_data_msgs::msg::ControlReport>();

    const std::string &create_time = get_current_time();
    mission_report->set__create_time(create_time);

    mission_report->set__control_id(mission.mission_id);
    mission_report->set__control_code(mission.mission_code);

    mission_report->set__control_type(CONTROL_TYPE_MISSION);
    mission_report->set__response_code(status_code);

    const ktp_data_msgs::msg::ControlReport &&mission_report_moved = std::move(*(mission_report));

    this->notify_mission_publisher_->publish(mission_report_moved);
}