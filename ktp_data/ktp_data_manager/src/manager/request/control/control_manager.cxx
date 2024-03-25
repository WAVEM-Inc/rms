#include "manager/request/control/control_manager.hxx"

ktp::data::ControlManager::ControlManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->graph_list_manager_ = std::make_shared<ktp::data::GraphListManager>(this->node_);

    this->assign_control_from_itf_service_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assign_control_from_itf_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignControl>(
        ASSIGN_CONTROL_FROM_ITF_SERVICE_NAME,
        std::bind(&ktp::data::ControlManager::assign_control_from_itf_service_cb, this, _1, _2, _3),
        rmw_qos_profile_services_default,
        this->assign_control_from_itf_service_cb_group_
        );

    this->assign_control_to_task_ctrl_service_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assign_control_to_task_ctrl_service_client_ = this->node_->create_client<ktp_data_msgs::srv::AssignControl>(
        ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME,
        rmw_qos_profile_services_default,
        this->assign_control_to_task_ctrl_service_client_cb_group_
        );
}

ktp::data::ControlManager::~ControlManager()
{
}

void ktp::data::ControlManager::assign_control_from_itf_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Response> response)
{
    const std::string &control_code = request->control.control_code;

    if (control_code == "")
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Assign Control From ITF control_code is empty");
        return;
    }
    else if (control_code == CONTROL_CODE_GRAPH_SYNC)
    {
        RCLCPP_INFO(this->node_->get_logger(), "Assign Control From ITF control_code is [%s]", control_code.c_str());
        this->graph_list_manager_->path_graph_graph_request();
    }
    else
    {
        const bool &is_request_assign_control_to_task_ctrl_success = this->request_assign_control_to_task_ctrl(request);

        if (is_request_assign_control_to_task_ctrl_success)
        {
            response->set__result(true);
        }
        else
        {
            response->set__result(false);
        }
    }
}

bool ktp::data::ControlManager::request_assign_control_to_task_ctrl(ktp_data_msgs::srv::AssignControl::Request::SharedPtr request)
{
    const bool &is_assign_control_service_server_ready = this->assign_control_to_task_ctrl_service_client_->wait_for_service(std::chrono::seconds(1));

    if (is_assign_control_service_server_ready)
    {
        rclcpp::Client<ktp_data_msgs::srv::AssignControl>::FutureAndRequestId future_and_request_id = this->assign_control_to_task_ctrl_service_client_->async_send_request(request);

        const std::future_status &future_status = future_and_request_id.wait_for(std::chrono::milliseconds(750));

        if (future_status == std::future_status::ready)
        {
            RCLCPP_INFO(this->node_->get_logger(), "success to assign control request");
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "failed to request...aborting");
            return false;
        }
        return true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "service server is not ready...aborting");
        return false;
    }
}