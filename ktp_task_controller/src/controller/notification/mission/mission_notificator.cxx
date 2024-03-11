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

void ktp::controller::MissionNotificator::notify_mission_status(uint8_t status_code, ktp_data_msgs::msg::MissionTask mission_task, int mission_task_index)
{
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Notify Mission Status -----------------------");

    RCLCPP_INFO(this->node_->get_logger(), "\n\tSTATUS CODE : [%d]\n\tTASK INDEX : [%d]", status_code, mission_task_index);

    ktp_data_msgs::msg::ServiceStatus::UniquePtr service_status = std::make_unique<ktp_data_msgs::msg::ServiceStatus>();

    const std::string &create_time = get_current_time();
    service_status->set__create_time(create_time);

    service_status->set__mission_code(MISSION_CODE);

    const std::string device_id = DEVICE_ID;
    const std::string &mission_id = device_id + create_time;
    service_status->set__mission_id(mission_id);

    service_status->set__owner(MISSION_OWNER);

    // -------------------------- 미션 상세 태스크 정보(task) --------------------------
    std::vector<ktp_data_msgs::msg::ServiceStatusTask> service_status_task_vec;
    ktp_data_msgs::msg::ServiceStatusTask::UniquePtr service_status_task = std::make_unique<ktp_data_msgs::msg::ServiceStatusTask>();

    const std::string &task_id = mission_task.task_id + std::to_string(mission_task_index);
    service_status_task->set__task_id(task_id);

    const std::string &task_code = mission_task.task_code;
    service_status_task->set__task_code(task_code);

    std::string status = "";

    switch (status_code)
    {
    case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_STARTED_CODE:
        status = MISSION_TASK_STARTED_STATUS;
        break;
    case ROUTE_TO_POSE_FEEDBACK_ON_PROGRESS_CODE:
        status = MISSION_TASK_ON_PROGRESS_STATUS;
        break;
    case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_SUCCEEDED_CODE:
        status = MISSION_TASK_END_STATUS;
        break;
    case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_CANCELED_CODE:
        status = MISSION_TASK_CANCELLED_STATUS;
        break;
    default:
        break;
    }

    service_status_task->set__status(status);
    service_status_task->set__seq(mission_task_index);

    // -------------------------- 미션 상세 태스크 정보(task) --------------------------

    // -------------------------- 태스크 상세 데이터(task_data) --------------------------
    ktp_data_msgs::msg::ServiceStatusTaskData::UniquePtr service_status_task_data = std::make_unique<ktp_data_msgs::msg::ServiceStatusTaskData>();
    
    const std::string &map_id = mission_task.task_data.map_id;
    service_status_task_data->set__map_id(map_id);

    const std::vector<std::string> &goal_vec = mission_task.task_data.goal;
    service_status_task_data->set__goal(goal_vec);

    const std::string &source = mission_task.task_data.source;
    service_status_task_data->set__source(source);

    // -------------------------- 태스크 상세 데이터(task_data) --------------------------

    const ktp_data_msgs::msg::ServiceStatusTaskData &&service_status_task_data_moved = std::move(*(service_status_task_data));
    service_status_task->set__task_data(service_status_task_data_moved);
    
    const ktp_data_msgs::msg::ServiceStatusTask &&service_status_task_moved = std::move(*(service_status_task));
    service_status_task_vec.push_back(service_status_task_moved);

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