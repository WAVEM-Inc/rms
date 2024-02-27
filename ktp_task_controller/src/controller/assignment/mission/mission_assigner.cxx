#include "controller/assignment/mission/mission_assigner.hxx"

ktp::controller::MissionAssigner::MissionAssigner(rclcpp::Node::SharedPtr node)
    : node_(node),
      mission_task_data_current_idx_(DEFAULT_INT),
      mission_task_data_last_idx_(DEFAULT_INT)
{
    this->mission_ = std::make_shared<ktp::domain::Mission>();

    this->assign_mission_service_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->assign_mission_service_ = this->node_->create_service<ktp_data_msgs::srv::AssignMission>(
        ASSIGN_MISSION_SERVICE_NAME,
        std::bind(&ktp::controller::MissionAssigner::assign_mission_service_cb, this, _1, _2, _3),
        rmw_qos_profile_services_default,
        this->assign_mission_service_cb_group_);

    this->ublox_fix_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ublox_fix_subscription_opts;
    ublox_fix_subscription_opts.callback_group = this->ublox_fix_subscription_cb_group_;
    this->ublox_fix_subscription_ = this->node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        UBLOX_FIX_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::controller::MissionAssigner::ublox_fix_subscription_cb, this, _1),
        ublox_fix_subscription_opts);

    this->path_graph_path_service_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->path_graph_path_service_client_ = this->node_->create_client<path_graph_msgs::srv::Path>(
        PATH_GRAPH_PATH_SERVICE_NAME,
        rmw_qos_profile_services_default,
        this->path_graph_path_service_client_cb_group_);

    this->route_to_pose_action_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const rcl_action_client_options_t &route_to_pose_action_client_opts = rcl_action_client_get_default_options();
    this->route_to_pose_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->node_,
        ROUTE_TO_POSE_ACTION_NAME,
        this->route_to_pose_action_client_cb_group_,
        route_to_pose_action_client_opts);
}

ktp::controller::MissionAssigner::~MissionAssigner()
{
}

void ktp::controller::MissionAssigner::assign_mission_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response)
{
    const ktp_data_msgs::msg::Mission &mission = request->mission;

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Assign Mission Service CB -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "\n\trequest_time : [%s]\n\tmission_id : [%s]\n\towner : [%s]\n\tmission_code : [%s]",
        CSTR(mission.request_time),
        CSTR(mission.mission_id),
        CSTR(mission.owner),
        CSTR(mission.mission_code));

    const std::vector<ktp_data_msgs::msg::MissionTask> &mission_task_vec = mission.task;
    for (const ktp_data_msgs::msg::MissionTask &mission_task : mission_task_vec)
    {
        RCLCPP_INFO(
            this->node_->get_logger(),
            "MissionTask Info\n\ttask_id : [%s]\n\ttask_code : [%s]\n\tseq : [%d]",
            CSTR(mission_task.task_id),
            CSTR(mission_task.task_code),
            mission_task.seq);

        const ktp_data_msgs::msg::MissionTaskData &mission_task_data = mission_task.task_data;
        RCLCPP_INFO(
            this->node_->get_logger(),
            "MissionTaskData Info\n\tmap_id : [%s]\n\tsource : [%s]",
            CSTR(mission_task_data.map_id),
            CSTR(mission_task_data.source));

        const std::vector<std::string> &goal_vec = mission_task_data.goal;

        for (const std::string &goal : goal_vec)
        {
            RCLCPP_INFO(this->node_->get_logger(), "\n\tgoal : [%s]", CSTR(goal));
        }

        this->mission_task_data_vec_.push_back(mission_task_data);
    }
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");

    this->mission_->set__mission(mission);

    response->set__result(true);
}

void ktp::controller::MissionAssigner::ublox_fix_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_cb)
{
    this->ublox_fix_cb_ = ublox_fix_cb;

    if (this->ublox_fix_cb_ != nullptr)
    {
        this->ublox_fix_cb_flag_ = true;
    }
    else
    {
        this->ublox_fix_cb_flag_ = false;
        return;
    }
}

path_graph_msgs::msg::Path ktp::controller::MissionAssigner::path_graph_path_service_req(ktp_data_msgs::msg::MissionTaskData mission_task_data)
{
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Path Graph Path Request ------------------------");

    path_graph_msgs::srv::Path::Request::SharedPtr path_request = std::make_shared<path_graph_msgs::srv::Path::Request>();
    path_request->set__start_node(mission_task_data.source);

    const std::vector<std::string> &goal_vec = mission_task_data.goal;
    const int &goal_vec_size = goal_vec.size();

    if (goal_vec_size > 1)
    {
        RCLCPP_WARN(this->node_->get_logger(), "");
        path_request->set__end_node(goal_vec[goal_vec_size - 1]);
    }
    else
    {
        path_request->set__end_node(goal_vec[DEFAULT_INT]);
    }

    if (this->ublox_fix_cb_flag_)
    {
        const double &longitude = this->ublox_fix_cb_->longitude;
        const double &latitude = this->ublox_fix_cb_->latitude;

        path_graph_msgs::msg::Position::UniquePtr position = std::make_unique<path_graph_msgs::msg::Position>();
        position->set__longitude(longitude);
        position->set__latitude(latitude);

        const path_graph_msgs::msg::Position &&position_moved = std::move(*(position));
        path_request->set__position(position_moved);

        const bool &is_service_server_ready = this->path_graph_path_service_client_->wait_for_service(std::chrono::seconds(1));

        if (is_service_server_ready)
        {
            rclcpp::Client<path_graph_msgs::srv::Path>::FutureAndRequestId future_and_request_id = this->path_graph_path_service_client_->async_send_request(path_request);
            const std::future_status &future_status = future_and_request_id.wait_for(std::chrono::milliseconds(750));

            if (future_status == std::future_status::ready)
            {
                RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
                RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Path Graph Path Response ---------------------------------------\n");

                const path_graph_msgs::srv::Path::Response::SharedPtr response = future_and_request_id.future.get();

                const path_graph_msgs::msg::Path &path = response->path;

                return path;
            }
            else
            {
                RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
                RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Path Graph Path Request -----------------------------------------\n");
                RCLCPP_ERROR(this->node_->get_logger(), "failed to request...aborting");
                RCLCPP_ERROR(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "path_graph path request ublox_fix is nullptr...aborting");
        (void)path_request;
    }

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}

ktp::domain::Mission::SharedPtr ktp::controller::MissionAssigner::transmiss_mission_to_notification()
{
    return this->mission_;
}

void ktp::controller::MissionAssigner::route_to_pose_send_goal()
{
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Route to Pose Send Goal ------------------------");

    const bool &is_mission_task_data_vec_empty = this->mission_task_data_vec_.empty();

    if (is_mission_task_data_vec_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose send goal mission_task_data_vec is empty...aborting");
        return;
    }

    const bool &is_route_to_pose_action_server_ready = this->route_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(1));

    if (!is_route_to_pose_action_server_ready)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose action server is not ready yet...aborting");
        return;
    }

    this->mission_task_data_last_idx_ = this->mission_task_data_vec_.size() - 1;

    const ktp_data_msgs::msg::MissionTaskData &current_mission_task_data = this->mission_task_data_vec_[mission_task_data_current_idx_];

    const path_graph_msgs::msg::Path &route_path = this->path_graph_path_service_req(current_mission_task_data);

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}