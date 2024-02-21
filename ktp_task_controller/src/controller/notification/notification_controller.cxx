#include "controller/notification/notification_controller.hxx"

ktp::controller::NotificationController::NotificationController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->assignment_controller_ = std::make_shared<ktp::controller::AssignmentController>(this->node_);

    this->notificate_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->notificate_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(NOTIFICATE_PUBLISHING_RATE),
        std::bind(&ktp::controller::NotificationController::notificate_timer_cb, this),
        this->notificate_control_publisher_cb_group_);

    this->notificate_mission_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notificate_mission_publihser_opts;
    notificate_mission_publihser_opts.callback_group = this->notificate_mission_publisher_cb_group_;
    this->notificate_mission_publihser_ = this->node_->create_publisher<ktp_data_msgs::msg::Mission>(
        NOTIFICATE_MISSION_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notificate_mission_publihser_opts);

    this->notificate_control_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notificate_control_publihser_opts;
    notificate_control_publihser_opts.callback_group = this->notificate_control_publisher_cb_group_;
    this->notificate_control_publihser_ = this->node_->create_publisher<ktp_data_msgs::msg::Control>(
        NOTIFICATE_CONTROL_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notificate_control_publihser_opts);
}

ktp::controller::NotificationController::~NotificationController()
{
}

void ktp::controller::NotificationController::notificate_timer_cb()
{
    const ktp_data_msgs::msg::Mission &mission = this->assignment_controller_->transmiss_mission_to_notification()->get__mission();

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

    const ktp_data_msgs::msg::Control &control = this->assignment_controller_->transmiss_control_to_notification()->get__control();

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "--------------------- Notificate Control Publish -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "\n\trequest_time : [%s]\n\tcontrol_id : [%s]\n\towner : [%s]\n\tcontrol_code : [%s]",
        CSTR(control.request_time),
        CSTR(control.control_id),
        CSTR(control.owner),
        CSTR(control.control_code));
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    this->notificate_control_publihser_->publish(control);
}