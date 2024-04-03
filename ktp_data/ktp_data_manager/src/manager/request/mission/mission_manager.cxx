#include "manager/request/mission/mission_manager.hxx"

ktp::data::MissionManager::MissionManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{

    this->assign_mission_from_itf_service_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assign_mission_from_itf_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignMission>(
        ASSIGN_MISSION_FROM_ITF_SERVICE_NAME,
        std::bind(&ktp::data::MissionManager::assign_mission_from_itf_service_cb, this, _1, _2, _3),
        rmw_qos_profile_services_default,
        this->assign_mission_from_itf_service_cb_group_);

    this->assign_mission_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assign_mission_client_ = this->node_->create_client<ktp_data_msgs::srv::AssignMission>(
        ASSIGN_MISSION_TO_TASK_CTRL_SERVICE_NAME,
        rmw_qos_profile_services_default,
        this->assign_mission_client_cb_group_);
}

ktp::data::MissionManager::~MissionManager()
{
}

void ktp::data::MissionManager::assign_mission_from_itf_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response)
{
    const ktp_data_msgs::msg::Mission &mission_request_from_itf = request->mission;

    const bool &assign_mission_result = this->assign_mission_service_req(mission_request_from_itf);

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------------------\n");
    RCLCPP_INFO(this->node_->get_logger(), "----------------------------- Assign Mission From Interface Response ---------------------------------\n");

    if (assign_mission_result)
    {
        RCLCPP_INFO(this->node_->get_logger(), "assignment mission from interface succeeded result : [%d]", assign_mission_result);
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "assignment mission from interface failed result : [%d]", assign_mission_result);
    }

    response->set__result(true);

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------------------\n");
}

bool ktp::data::MissionManager::assign_mission_service_req(const ktp_data_msgs::msg::Mission mission_request_from_itf)
{
    ktp_data_msgs::srv::AssignMission::Request::SharedPtr request = std::make_shared<ktp_data_msgs::srv::AssignMission::Request>();

    request->set__mission(mission_request_from_itf);

    const bool &is_service_server_ready = this->assign_mission_client_->wait_for_service(std::chrono::seconds(1));

    if (is_service_server_ready)
    {
        rclcpp::Client<ktp_data_msgs::srv::AssignMission>::FutureAndRequestId future_and_request_id = this->assign_mission_client_->async_send_request(request);
        const std::future_status &future_status = future_and_request_id.wait_for(std::chrono::milliseconds(750));

        if (future_status == std::future_status::ready)
        {
            RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------------------\n");
            RCLCPP_INFO(this->node_->get_logger(), "----------------------------- Assign Mission To Controller Response ----------------------------------\n");

            const ktp_data_msgs::srv::AssignMission::Response::SharedPtr response = future_and_request_id.future.get();
            const bool &result = response->result;

            if (result)
            {
                RCLCPP_INFO(this->node_->get_logger(), "assignment mission to controller succeeded result : [%d]", result);
            }
            else
            {
                RCLCPP_ERROR(this->node_->get_logger(), "assignment mission to controller failed result : [%d]", result);
            }

            RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------------------\n");

            return result;
        }
        else
        {
            RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------------------\n");
            RCLCPP_ERROR(this->node_->get_logger(), "----------------------------- Assign Mission To Controller Request ----------------------------------\n");
            RCLCPP_ERROR(this->node_->get_logger(), "failed to request...aborting");
            RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
            return false;
        }
    }
    else
    {
        RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------------------------------------\n");
        RCLCPP_ERROR(this->node_->get_logger(), "----------------------------- Assign Mission To Controller Request ----------------------------------\n");
        RCLCPP_ERROR(this->node_->get_logger(), "service server is not ready...aborting");
        RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
        return false;
    }
}