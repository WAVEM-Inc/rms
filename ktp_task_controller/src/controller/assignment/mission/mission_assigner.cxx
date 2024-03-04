#include "controller/assignment/mission/mission_assigner.hxx"

ktp::controller::MissionAssigner::MissionAssigner(rclcpp::Node::SharedPtr node)
    : node_(node),
      mission_task_current_idx_(DEFAULT_INT),
      mission_task_last_idx_(DEFAULT_INT),
      mission_task_path_current_idx_(DEFAULT_INT),
      mission_task_path_last_idx_(DEFAULT_INT),
      node_current_idx_(DEFAULT_INT),
      node_last_idx_(DEFAULT_INT)
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
    this->route_to_pose_action_client_ = rclcpp_action::create_client<route_msgs::action::RouteToPose>(
        this->node_,
        ROUTE_TO_POSE_ACTION_NAME,
        this->route_to_pose_action_client_cb_group_,
        route_to_pose_action_client_opts);

    this->route_to_pose_status_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions route_to_pose_status_subscription_opts;
    route_to_pose_status_subscription_opts.callback_group = this->route_to_pose_status_subscription_cb_group_;
    this->route_to_pose_status_subscription_ = this->node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        ROUTE_TO_POSE_STATUS_TOPIC_NAME,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::controller::MissionAssigner::route_to_pose_subscription_cb, this, _1),
        route_to_pose_status_subscription_opts);
}

ktp::controller::MissionAssigner::~MissionAssigner()
{
}

ktp::domain::Mission::SharedPtr ktp::controller::MissionAssigner::transmiss_mission_to_notification()
{
    return this->mission_;
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

void ktp::controller::MissionAssigner::assign_mission_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response)
{
    const bool &is_mission_task_vec_empty = this->mission_task_vec_.empty();

    if (!is_mission_task_vec_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Mission has already assigned...aborting");
        response->set__result(false);
        return;
    }

    const ktp_data_msgs::msg::Mission &mission = request->mission;

    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Assign Mission Service CB -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "Mission Info\n\trequest_time : [%s]\n\tmission_id : [%s]\n\towner : [%s]\n\tmission_code : [%s]",
        CSTR(mission.request_time),
        CSTR(mission.mission_id),
        CSTR(mission.owner),
        CSTR(mission.mission_code));

    this->mission_task_vec_ = mission.task;
    this->mission_task_last_idx_ = this->mission_task_vec_.size();

    RCLCPP_INFO(this->node_->get_logger(), "MissionTaskVec Size : [%d]", this->mission_task_last_idx_);

    this->count++;
    RCLCPP_INFO(this->node_->get_logger(), "COUNT : [%d]", this->count);

    printf("\n");

    this->mission_->set__mission(mission);
    response->set__result(true);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    const bool &is_mission_assigned = this->mission_ != nullptr;

    if (is_mission_assigned)
    {
        const bool &path_request_result = this->request_converting_goal_to_path();

        if (path_request_result)
        {
            this->mission_task_path_current_idx_ = this->mission_task_current_idx_;
            this->mission_task_path_last_idx_ = (int)this->mission_task_path_vec_.size();

            RCLCPP_INFO(this->node_->get_logger(), "Converting Goal to Path succeeded. Sending Goal... [%d]", this->mission_task_path_current_idx_);
            this->route_to_pose_send_goal();
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Converting Goal to Path failed...aborting");
            return;
        }
    }
    else
    {
        return;
    }
}

bool ktp::controller::MissionAssigner::request_converting_goal_to_path()
{
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Path Graph Path Request ------------------------");

    if (!(this->ublox_fix_cb_flag_))
    {
        const char *err_msg = "Goal to Path ublox_fix is nullptr...aborting";
        RCLCPP_ERROR(this->node_->get_logger(), "%s", err_msg);
        // throw ktp::exceptions::DataOmissionException(err_msg);

        return false;
    }

    for (auto it = this->mission_task_vec_.begin(); it != this->mission_task_vec_.end(); ++it)
    {
        const size_t &index = std::distance(this->mission_task_vec_.begin(), it);

        RCLCPP_INFO(this->node_->get_logger(), "Request Converting Goal to Path index : [%zu]", index);

        const ktp_data_msgs::msg::MissionTaskData &mission_task_data = this->mission_task_vec_[index].task_data;

        path_graph_msgs::srv::Path::Request::SharedPtr path_request = std::make_shared<path_graph_msgs::srv::Path::Request>();
        path_request->set__start_node(mission_task_data.source);

        const std::vector<std::string> &goal_vec = mission_task_data.goal;
        const int &goal_vec_size = (int)goal_vec.size();

        if (goal_vec_size > 1)
        {
            RCLCPP_WARN(this->node_->get_logger(), "Goal to Path MissionTask's Goal list's size is over than 1, It will proceed last index goal");
            path_request->set__end_node(goal_vec[goal_vec_size - 1]);
        }
        else
        {
            path_request->set__end_node(goal_vec[DEFAULT_INT]);
        }

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
                RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Path Graph Path Response ---------------------------------------\n");

                const path_graph_msgs::srv::Path::Response::SharedPtr response = future_and_request_id.future.get();
                const route_msgs::msg::Path &path = response->path;
                this->mission_task_path_vec_.push_back(path);

                RCLCPP_INFO(this->node_->get_logger(), "Goal to Path\n\tindex : [%zu]\n\tsize : [%zu]", index, this->mission_task_path_vec_.size());

                return true;
            }
            else
            {
                RCLCPP_ERROR(this->node_->get_logger(), "failed to request...aborting");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "service server is not ready...aborting");
            return false;
        }
    }
}

void ktp::controller::MissionAssigner::route_to_pose_send_goal()
{
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Route to Pose Send Goal ------------------------");

    const bool &is_path_vec_empty = this->mission_task_path_vec_.empty();

    if (is_path_vec_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose send goal path_vec is empty...aborting");
        return;
    }

    const bool &is_route_to_pose_action_server_ready = this->route_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(1));

    if (!is_route_to_pose_action_server_ready)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose action server is not ready yet...aborting");
        return;
    }

    this->mission_task_last_idx_ = this->mission_task_vec_.size() - 1;

    route_msgs::action::RouteToPose::Goal::UniquePtr goal = std::make_unique<route_msgs::action::RouteToPose::Goal>();

    const std::vector<route_msgs::msg::Node> &node_list = this->mission_task_path_vec_[this->mission_task_path_current_idx_].node_list;
    this->node_last_idx_ = (int)node_list.size();

    const route_msgs::msg::Node &start_node = node_list[this->node_current_idx_];
    goal->set__start_node(start_node);

    const route_msgs::msg::Node &end_node = node_list[this->node_last_idx_];
    goal->set__end_node(end_node);

    rclcpp_action::Client<route_msgs::action::RouteToPose>::SendGoalOptions goal_opts = rclcpp_action::Client<route_msgs::action::RouteToPose>::SendGoalOptions();
    goal_opts.goal_response_callback = std::bind(&ktp::controller::MissionAssigner::route_to_pose_goal_response_cb, this, _1);
    goal_opts.feedback_callback = std::bind(&ktp::controller::MissionAssigner::route_to_pose_feedback_cb, this, _1, _2);
    goal_opts.result_callback = std::bind(&ktp::controller::MissionAssigner::route_to_pose_result_cb, this, _1);

    const route_msgs::action::RouteToPose::Goal &&goal_moved = std::move(*(goal));

    std::shared_future<rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr> goal_future = this->route_to_pose_action_client_->async_send_goal(goal_moved);
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>> &goal_handle = goal_future.get();

    RCLCPP_INFO(
        this->node_->get_logger(),
        "route_to_pose goal sent\n\tpath_current_idx : [%d]\n\tnode_current_idx : [%d]",
        this->mission_task_path_current_idx_,
        this->node_current_idx_);

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}

void ktp::controller::MissionAssigner::route_to_pose_goal_response_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr &goal_handle)
{
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Route to Pose Goal Handle -----------------------");

    if (!goal_handle)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "goal handle aborted...");
    }
    else
    {
        RCLCPP_INFO(this->node_->get_logger(), "goal handle aceepted!");
    }

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}

void ktp::controller::MissionAssigner::route_to_pose_feedback_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr goal_handle_ptr, const std::shared_ptr<const route_msgs::action::RouteToPose::Feedback> feedback_ptr)
{
    const int8_t &goal_status = goal_handle_ptr->get_status();

    RCLCPP_INFO(this->node_->get_logger(), "");
}

void ktp::controller::MissionAssigner::route_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::WrappedResult &wrapped_result)
{
}

void ktp::controller::MissionAssigner::route_to_pose_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr route_to_pose_status_cb)
{
    const bool &is_path_vec_empty = this->mission_task_path_vec_.empty();

    if (is_path_vec_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose send goal path_vec is empty...aborting");
        return;
    }
    else
    {
        const action_msgs::msg::GoalStatus &goal_status = route_to_pose_status_cb->status_list.back();
        const uint8_t &goal_status_code = goal_status.status;

        RCLCPP_INFO(
            this->node_->get_logger(),
            "route_to_pose status callback\n\twaypoints index : [%d]\n\twaypoints size : [%d]\n\tgoal_status_code : [%d]",
            this->mission_task_path_current_idx_,
            this->mission_task_path_last_idx_,
            goal_status_code);

        const bool &is_mission_task_path_ended = (this->mission_task_path_current_idx_ == (this->mission_task_path_last_idx_ - 1));
    }
}