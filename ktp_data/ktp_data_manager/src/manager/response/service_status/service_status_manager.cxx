#include "manager/response/service_status/service_status_manager.hxx"

ktp::data::ServiceStatusManager::ServiceStatusManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->service_status_from_task_ctrl_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions service_status_from_task_controller_subscription_opts;
    service_status_from_task_controller_subscription_opts.callback_group = this->service_status_from_task_ctrl_subscription_cb_group_;
    this->service_status_from_task_ctrl_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::ServiceStatus>(
        SERVICE_STATUS_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ServiceStatusManager::service_status_from_task_ctrl_subscription_cb, this, _1),
        service_status_from_task_controller_subscription_opts);

    this->service_status_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions service_status_to_interface_publisher_opts;
    service_status_to_interface_publisher_opts.callback_group = this->service_status_to_itf_publisher_cb_group_;
    this->service_status_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ServiceStatus>(
        SERVICE_STATUS_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        service_status_to_interface_publisher_opts);
}

ktp::data::ServiceStatusManager::~ServiceStatusManager()
{

}

void ktp::data::ServiceStatusManager::service_status_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ServiceStatus::SharedPtr service_status_cb)
{
    printf("-----------------------------------------------------------------------------------------------------\n");
    printf("-------------------------------------- Service Status -----------------------------------------------\n");
    const ktp_data_msgs::msg::ServiceStatus &service_status = *service_status_cb;
    RCLCPP_INFO(
        this->node_->get_logger(),
        "ServiceStatusManager ServiceStatus\n\tcreate_time : [%s]\n\tmission_code : [%s]\n\tmission_id : [%s]\n\towner : [%s]\n\treserve : [%s]",
        service_status.create_time.c_str(),
        service_status.mission_code.c_str(),
        service_status.mission_id.c_str(),
        service_status.owner.c_str(),
        service_status.reserve.c_str());

    const std::vector<ktp_data_msgs::msg::ServiceStatusTask> task_vec = service_status.task;
    for (const ktp_data_msgs::msg::ServiceStatusTask &task : task_vec)
    {
        RCLCPP_INFO(
            this->node_->get_logger(),
            "ServiceStatusManager ServiceStatus Task\n\ttask_id : [%s]\n\ttask_code : [%s]\n\ttask_code : [%s]\b\tstatus : [%s]\n\tseq : [%d]",
            task.task_id.c_str(),
            task.task_code.c_str(),
            task.status.c_str(),
            task.seq);

        const ktp_data_msgs::msg::ServiceStatusTaskData &task_data = task.task_data;
        RCLCPP_INFO(
            this->node_->get_logger(),
            "ServiceStatusManager ServiceStatus TaskData\n\tmap_id : [%s]\n\tsource : [%s]",
            task_data.map_id.c_str(),
            task_data.source.c_str());

        const std::vector<std::string> &goal_vec = task_data.goal;

        for (const std::string &goal : goal_vec)
        {
            RCLCPP_INFO(
                this->node_->get_logger(),
                "ServiceStatusManager ServiceStatus TaskData\n\tgoal : [%s]",
                goal.c_str());
        }
    }

    this->service_status_to_itf_publisher_->publish(service_status);
    printf("-----------------------------------------------------------------------------------------------------\n");
}