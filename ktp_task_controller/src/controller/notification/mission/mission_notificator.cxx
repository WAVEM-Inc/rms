#include "controller/notification/mission/mission_notificator.hxx"

ktp::controller::MissionNotificator::MissionNotificator(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->notify_mission_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notify_mission_status_publisher_opts;
    notify_mission_status_publisher_opts.callback_group = this->notify_mission_status_publisher_cb_group_;
    this->notify_mission_status_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ServiceStatus>(
        NOTIFY_MISSION_STATUS_TO_MGR_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        notify_mission_status_publisher_opts);

    this->notify_robot_navigation_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions notify_robot_navigation_status_publisher_opts;
    notify_robot_navigation_status_publisher_opts.callback_group = this->notify_robot_navigation_status_publisher_cb_group_;
    this->notify_robot_navigation_status_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::Status>(
            NOTIFY_ROBOT_NAVIGATION_STATUS_TO_MGR_TOPIC,
            rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
            notify_robot_navigation_status_publisher_opts);
}

ktp::controller::MissionNotificator::~MissionNotificator()
{
}

void ktp::controller::MissionNotificator::notify_mission_status(int32_t status_code, ktp_data_msgs::msg::MissionTask mission_task)
{
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Notify Mission Status -----------------------");

    RCLCPP_INFO(this->node_->get_logger(), "\n\tSTATUS CODE : [%d]", status_code);

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

    const std::string &task_id = mission_task.task_id + std::to_string(DEFAULT_INT);
    service_status_task->set__task_id(task_id);

    const std::string &task_code = mission_task.task_code;
    service_status_task->set__task_code(task_code);

    std::string status = "";

    switch (status_code)
    {
    case MISSION_TASK_STARTED_CODE:
        status = MISSION_TASK_STARTED_STATUS;
        break;
    case MISSION_TASK_SOURCE_ARRIVED_CODE:
        status = MISSION_TASK_SOURCE_ARRIVED_STATUS;
        break;
    case MISSION_TASK_TAKEN_CODE:
        status = MISSION_TASK_TAKEN_STATUS;
        break;
    case MISSION_TASK_ON_PROGRESS_CODE:
        status = MISSION_TASK_ON_PROGRESS_STATUS;
        break;
    case MISSION_TASK_DEST_ARRIVED_CODE:
        status = MISSION_TASK_DEST_ARRIVED_STATUS;
        break;
    case MISSION_TASK_ENDED_CODE:
        status = MISSION_TASK_END_STATUS;
        break;
    case MISSION_TASK_CANCELLED_CODE:
        status = MISSION_TASK_CANCELLED_STATUS;
        break;
    case MISSION_TASK_FAILED_CODE:
        status = MISSION_TASK_FAILED_STATUS;
        break;
    default:
        break;
    }

    service_status_task->set__status(status);
    service_status_task->set__seq(DEFAULT_INT);

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

void ktp::controller::MissionNotificator::notify_robot_navigation_status(ktp::domain::NavigationStatus navigation_status)
{
    ktp_data_msgs::msg::Status::UniquePtr robot_navigation_status = std::make_unique<ktp_data_msgs::msg::Status>();

    const ktp_data_msgs::msg::MissionTask &mission_task = navigation_status.get__mission_task();

    const std::string &map_id = mission_task.task_data.map_id;
    robot_navigation_status->set__map_id(map_id);

    const int &drive_status = navigation_status.get__drive_status();
    robot_navigation_status->set__drive_status(static_cast<int16_t>(drive_status));

    const route_msgs::msg::Node &start_node = navigation_status.get__start_node();
    robot_navigation_status->set__from_node(start_node.node_id);

    const route_msgs::msg::Node &end_node = navigation_status.get__end_node();
    robot_navigation_status->set__to_node(end_node.node_id);

    const ktp_data_msgs::msg::Status &&robot_navigation_status_moved = std::move(*(robot_navigation_status));

    this->notify_robot_navigation_status_publisher_->publish(robot_navigation_status_moved);
}