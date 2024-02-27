#include "manager/request/mission/mission_manager.hxx"

ktp::data::MissionManager::MissionManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->mission_from_itf_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mission_from_itf_subscription_opts;
    mission_from_itf_subscription_opts.callback_group = this->mission_from_itf_subscription_cb_group_;
    this->mission_from_itf_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Mission>(
        MISSION_FROM_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::MissionManager::mission_from_itf_subscription_cb, this, _1),
        mission_from_itf_subscription_opts);

    this->assign_mission_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assign_mission_client_ = this->node_->create_client<ktp_data_msgs::srv::AssignMission>(
        MISSION_TO_TASK_CTRL_SERVICE_NAME,
        rmw_qos_profile_services_default,
        this->assign_mission_client_cb_group_);
}

ktp::data::MissionManager::~MissionManager()
{
}

void ktp::data::MissionManager::mission_from_itf_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb)
{
    this->assign_mission_service_req(mission_cb);
}

void ktp::data::MissionManager::assign_mission_service_req(const ktp_data_msgs::msg::Mission::SharedPtr mission_from_itf)
{
    ktp_data_msgs::srv::AssignMission::Request::SharedPtr request = std::make_shared<ktp_data_msgs::srv::AssignMission::Request>();

    request->set__mission(*(mission_from_itf));

    const bool &is_service_server_ready = this->assign_mission_client_->wait_for_service(std::chrono::seconds(1));

    if (is_service_server_ready)
    {
        rclcpp::Client<ktp_data_msgs::srv::AssignMission>::FutureAndRequestId future_and_request_id = this->assign_mission_client_->async_send_request(request);
        const std::future_status &future_status = future_and_request_id.wait_for(std::chrono::milliseconds(750));

        if (future_status == std::future_status::ready)
        {
            RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
            RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Assign Mission Response ----------------------------------------\n");

            const ktp_data_msgs::srv::AssignMission::Response::SharedPtr response = future_and_request_id.future.get();
            const bool &result = response->result;

            if (result == true)
            {
                RCLCPP_INFO(this->node_->get_logger(), "assignment succeeded result : [%d]", result);
            }
            else
            {
                RCLCPP_ERROR(this->node_->get_logger(), "assignment failed result : [%d]", result);
            }

            RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
            RCLCPP_ERROR(this->node_->get_logger(), "------------------------------------ Assign Mission Request -----------------------------------------\n");
            RCLCPP_ERROR(this->node_->get_logger(), "failed to request...aborting");
            RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
            return;
        }
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
        RCLCPP_ERROR(this->node_->get_logger(), "------------------------------------ Assign Mission Request -----------------------------------------\n");
        RCLCPP_ERROR(this->node_->get_logger(), "service server is not ready...aborting");
        RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
        return;
    }
}