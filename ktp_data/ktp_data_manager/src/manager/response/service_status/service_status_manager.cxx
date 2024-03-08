#include "manager/response/service_status/service_status_manager.hxx"

ktp::data::ServiceStatusManager::ServiceStatusManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_status_from_task_ctrl_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_status_from_task_ctrl_subscription_opts;
    mission_status_from_task_ctrl_subscription_opts.callback_group = this->mission_status_from_task_ctrl_subscription_cb_group_;
    this->mission_status_from_task_ctrl_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Mission>(
        MISSION_STATUS_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ServiceStatusManager::mission_status_from_task_ctrl_subscription_cb, this, _1),
        mission_status_from_task_ctrl_subscription_opts);

    this->service_status_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions service_status_to_itf_publisher_opts;
    service_status_to_itf_publisher_opts.callback_group = this->service_status_to_itf_publisher_cb_group_;
    this->service_status_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ServiceStatus>(
        SERVICE_STATUS_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        service_status_to_itf_publisher_opts);
}

ktp::data::ServiceStatusManager::~ServiceStatusManager()
{
}

void ktp::data::ServiceStatusManager::mission_status_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb)
{
    RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
    RCLCPP_INFO(this->node_->get_logger(), "--------------------------- Nofiticate Mission(ServiceStatus) Callback ------------------------------\n");

    ktp_data_msgs::msg::ServiceStatus::UniquePtr service_status = std::make_unique<ktp_data_msgs::msg::ServiceStatus>();

    const std::string &create_time = get_current_time();
    service_status->set__create_time(create_time);

    service_status->set__mission_code(mission_cb->mission_code);
    service_status->set__mission_id(mission_cb->mission_id);
    service_status->set__owner(mission_cb->owner);

    RCLCPP_INFO(
        this->node_->get_logger(),
        "ServiceStatusManager ServiceStatus\n\tcreate_time : [%s]\n\tmission_code : [%s]\n\tmission_id : [%s]\n\towner : [%s]",
        CSTR(service_status->create_time),
        CSTR(service_status->mission_code),
        CSTR(service_status->mission_id),
        CSTR(service_status->owner));

    std::vector<ktp_data_msgs::msg::ServiceStatusTask> service_status_task_vec;

    const std::vector<ktp_data_msgs::msg::MissionTask> &mission_task_vec = mission_cb->task;
    for (const ktp_data_msgs::msg::MissionTask &mission_task : mission_task_vec)
    {
        ktp_data_msgs::msg::ServiceStatusTask::UniquePtr service_status_task = std::make_unique<ktp_data_msgs::msg::ServiceStatusTask>();
        service_status_task->set__task_id(mission_task.task_id);
        service_status_task->set__task_code(mission_task.task_code);
        service_status_task->set__seq(mission_task.seq);

        const ktp_data_msgs::msg::MissionTaskData &mission_task_data = mission_task.task_data;
        ktp_data_msgs::msg::ServiceStatusTaskData::UniquePtr service_status_task_data = std::make_unique<ktp_data_msgs::msg::ServiceStatusTaskData>();
        service_status_task_data->set__map_id(mission_task_data.map_id);
        service_status_task_data->set__goal(mission_task_data.goal);
        service_status_task_data->set__source(mission_task_data.source);

        const ktp_data_msgs::msg::ServiceStatusTask &&service_status_task_moved = std::move(*(service_status_task));
        service_status_task_vec.push_back(service_status_task_moved);
    }

    service_status->set__task(service_status_task_vec);
    
    for (const ktp_data_msgs::msg::ServiceStatusTask &service_status_task : service_status->task)
    {
        RCLCPP_INFO(
            this->node_->get_logger(),
            "ServiceStatusManager ServiceStatus Task\n\ttask_id : [%s]\n\ttask_code : [%s]\n\tstatus : [%s]\n\tseq : [%d]",
            CSTR(service_status_task.task_id),
            CSTR(service_status_task.task_code),
            CSTR(service_status_task.status),
            service_status_task.seq);

        const ktp_data_msgs::msg::ServiceStatusTaskData &service_status_task_data = service_status_task.task_data;
        RCLCPP_INFO(
            this->node_->get_logger(),
            "ServiceStatusManager ServiceStatus TaskData\n\tmap_id : [%s]\n\tsource : [%s]",
            CSTR(service_status_task_data.map_id),
            CSTR(service_status_task_data.source));

        const std::vector<std::string> &goal_vec = service_status_task_data.goal;

        for (const std::string &goal : goal_vec)
        {
            RCLCPP_INFO(this->node_->get_logger(), "ServiceStatusManager ServiceStatus TaskData\n\tgoal : [%s]", CSTR(goal));
        }
    }

    const ktp_data_msgs::msg::ServiceStatus &&service_status_moved = std::move(*(service_status));
    this->service_status_to_itf_publisher_->publish(service_status_moved);
    RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
}