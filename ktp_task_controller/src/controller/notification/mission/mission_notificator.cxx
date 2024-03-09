#include "controller/notification/mission/mission_notificator.hxx"

ktp::controller::MissionNotificator::MissionNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->notify_mission_report_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notify_mission_report_publisher_opts;
    notify_mission_report_publisher_opts.callback_group = this->notify_mission_report_publisher_cb_group_;
    this->notify_mission_report_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ControlReport>(
        NOTIFY_MISSION_REPORT_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notify_mission_report_publisher_opts);

    this->notify_mission_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notify_mission_status_publisher_opts;
    notify_mission_status_publisher_opts.callback_group = this->notify_mission_status_publisher_cb_group_;
    this->notify_mission_status_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ServiceStatus>(
        NOTIFY_MISSION_STATUS_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notify_mission_status_publisher_opts);
}

ktp::controller::MissionNotificator::~MissionNotificator()
{
}

void ktp::controller::MissionNotificator::notify_mission_status(ktp::domain::Mission::SharedPtr domain_mission)
{
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Notify Mission Status -----------------------");

    const uint8_t &status_code = domain_mission->get__status_code();
    RCLCPP_INFO(this->node_->get_logger(), "\n\tstatus_code : [%d]", status_code);

    const ktp_data_msgs::msg::Mission &mission = domain_mission->get__mission();

    ktp_data_msgs::msg::ServiceStatus::UniquePtr service_status = std::make_unique<ktp_data_msgs::msg::ServiceStatus>();

    const std::string &create_time = get_current_time();
    service_status->set__create_time(create_time);

    service_status->set__mission_code(MISSION_CODE);

    const std::string device_id = "1234123512";
    const std::string &mission_id = device_id + create_time;
    service_status->set__mission_id(mission_id);

    service_status->set__owner(MISSION_OWNER);

    const std::vector<ktp_data_msgs::msg::MissionTask> &mission_task_vec = mission.task;

    std::vector<ktp_data_msgs::msg::ServiceStatusTask> service_status_task_vec;
    ktp_data_msgs::msg::ServiceStatusTask::UniquePtr service_status_task = std::make_unique<ktp_data_msgs::msg::ServiceStatusTask>();

    for (const ktp_data_msgs::msg::MissionTask &mission_task : mission_task_vec)
    {
        const std::string &task_id = mission_task.task_id;
        service_status_task->set__task_id(task_id);

        const std::string &task_code = mission_task.task_code;
        service_status_task->set__task_code(task_code);

        std::string status = "";

        switch (status_code)
        {
        // 1001 - 차량 출발
        case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_STARTED_CODE:
            status = MISSION_TASK_STARTED_STATUS;
            break;
        // 2001 - 작업 중(이동 중)
        case ROUTE_TO_POSE_FEEDBACK_ON_PROGRESS_CODE:
            status = MISSION_TASK_STARTED_STATUS;
            break;
        // 3001 - 장애물 감지(LiDAR)
        case ROUTE_TO_POSE_FEEDBACK_LIDAR_OBJECT_DETECTED_CODE:
            status = MISSION_TASK_STARTED_STATUS;
            break;
        // 3002 - 장애물 감지(협력 주행)
        case ROUTE_TO_POSE_FEEDBACK_COOPERATIVE_OBJECT_DETECTED_CODE:
            status = MISSION_TASK_STARTED_STATUS;
            break;
        // 4001 - 이동 완료
        case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_SUCCEEDED_CODE:
            status = MISSION_TASK_STARTED_STATUS;
            break;
        // 5001 - 이동 취소
        case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_CANCELED_CODE:
            status = MISSION_TASK_STARTED_STATUS;
            break;
        default:
            break;
        }

        service_status_task->set__status(status);

        const ktp_data_msgs::msg::ServiceStatusTask &&service_status_task_moved = std::move(*(service_status_task));
        service_status_task_vec.push_back(service_status_task_moved);
    }

    service_status->set__task(service_status_task_vec);

    const ktp_data_msgs::msg::ServiceStatus &&service_status_moved = std::move(*(service_status));
    this->notify_mission_status_publisher_->publish(service_status_moved);
}


void ktp::controller::MissionNotificator::notify_mission_report(ktp::domain::Mission::SharedPtr domain_mission)
{
    const ktp_data_msgs::msg::Mission &mission = domain_mission->get__mission();

    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Notify Mission Response -----------------------");

    ktp_data_msgs::msg::ControlReport::UniquePtr mission_report = std::make_unique<ktp_data_msgs::msg::ControlReport>();

    const std::string &create_time = get_current_time();
    mission_report->set__create_time(create_time);

    mission_report->set__control_id(mission.mission_id);
    mission_report->set__control_type(CONTROL_TYPE_MISSION);
    mission_report->set__control_code(mission.mission_code);

    const uint8_t &repsonse_code = domain_mission->get__response_code();
    RCLCPP_INFO(this->node_->get_logger(), "\n\trepsonse_code : [%d]", repsonse_code);

    mission_report->set__response_code(repsonse_code);

    const ktp_data_msgs::msg::ControlReport &&mission_report_moved = std::move(*(mission_report));

    this->notify_mission_report_publisher_->publish(mission_report_moved);
}