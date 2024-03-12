#include "controller/assignment/mission/mission_assigner.hxx"

ktp::controller::MissionAssigner::MissionAssigner(rclcpp::Node::SharedPtr node)
    : node_(node), task_current_index_(DEFAULT_INT),
      task_vec_size_(DEFAULT_INT),
      path_current_index_(DEFAULT_INT),
      path_vec_size_(DEFAULT_INT),
      node_current_index_(DEFAULT_INT),
      node_list_size_(DEFAULT_INT),
      schedule_index_(DEFAULT_INT)
{
    this->domain_mission_ = std::make_shared<ktp::domain::Mission>();
    this->mission_notificator_ = std::make_shared<ktp::controller::MissionNotificator>(this->node_);

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
}

ktp::controller::MissionAssigner::~MissionAssigner()
{
}

void ktp::controller::MissionAssigner::ublox_fix_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_cb)
{
    // ###########################################################################
    // ublox 콜백 전역화 및 NULL 포인터 유효성 검사
    // ###########################################################################
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
    // ###########################################################################
}

void ktp::controller::MissionAssigner::assign_mission_service_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
    const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response)
{
    // ###########################################################################
    // 미션 태스크 중복 할당 검사
    // ###########################################################################
    const bool &is_mission_task_vec_not_empty = !(this->task_vec_.empty());

    if (is_mission_task_vec_not_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Mission has already assigned...aborting");
        response->set__result(false);
        return;
    }
    // ###########################################################################

    // ###########################################################################
    // ublox 콜백 포인터 유효성 검사
    // ###########################################################################
    if (!(this->ublox_fix_cb_flag_))
    {
        const char *err_msg = "Goal to Path ublox_fix is nullptr...aborting";
        RCLCPP_ERROR(this->node_->get_logger(), "%s", err_msg);
        throw ktp::exceptions::DataOmissionException(err_msg);

        return;
    }
    // ###########################################################################

    // ###########################################################################
    // 미션 태스크 할당
    // ###########################################################################
    const ktp_data_msgs::msg::Mission &mission = request->mission;

    RCLCPP_INFO(this->node_->get_logger(), "---------------------- Assign Mission Service CB -----------------------");
    RCLCPP_INFO(
        this->node_->get_logger(),
        "Mission Info\n\trequest_time : [%s]\n\tmission_id : [%s]\n\towner : [%s]\n\tmission_code : [%s]",
        CSTR(mission.request_time),
        CSTR(mission.mission_id),
        CSTR(mission.owner),
        CSTR(mission.mission_code));

    this->task_vec_ = mission.task;
    this->task_vec_size_ = this->task_vec_.size();

    RCLCPP_INFO(this->node_->get_logger(), "MissionTaskVec Size : [%d]", this->task_vec_size_);
    // ###########################################################################

    // ###########################################################################
    // 임무 수신 성공 보고
    // ###########################################################################
    this->domain_mission_->set__response_code(MISSION_RECEPTION_SUCCEEDED_CODE);
    this->domain_mission_->set__mission(mission);

    this->mission_notificator_->notify_mission_report(this->domain_mission_);

    const bool &is_mission_assigned = this->domain_mission_ != nullptr;
    // ###########################################################################

    // ###########################################################################
    // [ 대기 장소 -> 출발지(상차지) ] 경로 조회
    // ###########################################################################
    const path_graph_msgs::srv::Path::Request::SharedPtr &waiting_area_to_top_chart_path_request = this->build_waiting_area_to_top_chart_path_request();
    const bool &is_waiting_area_to_top_chart_path_request = this->request_converting_goal_to_path(waiting_area_to_top_chart_path_request);
    // ###########################################################################

    // ###########################################################################
    // [ 대기 장소 -> 출발지(상차지) ] 경로 전송
    // ###########################################################################
    if (is_mission_assigned)
    {
        if (is_waiting_area_to_top_chart_path_request)
        {
            this->path_current_index_ = this->task_current_index_;
            this->path_vec_size_ = static_cast<int>(this->path_vec_.size());

            RCLCPP_INFO(this->node_->get_logger(), "Converting Goal to Path succeeded. Sending Goal... [%d]", this->path_current_index_);
            this->route_to_pose_send_goal();
            this->is_waiting_area_to_top_chart_proceeding = true;

            response->set__result(true);
        }
        else
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Converting Goal to Path failed...aborting");
            RCLCPP_INFO(this->node_->get_logger(), "Aborting Mission\n\ttask_current_index : [%d]", this->task_current_index_);
            this->mission_notificator_->notify_mission_status(MISSION_ASSIGN_FAILED_CODE, this->task_vec_[this->task_current_index_], this->task_current_index_);

            return;
        }
    }
    else
    {
        return;
    }
    // ###########################################################################
}

bool ktp::controller::MissionAssigner::request_converting_goal_to_path(path_graph_msgs::srv::Path::Request::SharedPtr path_request)
{
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Path Graph Path Request ------------------------");
    
    // ###########################################################################
    // ublox 콜백 포인터 유효성 검사
    // ###########################################################################
    if (!(this->ublox_fix_cb_flag_))
    {
        const char *err_msg = "Goal to Path ublox_fix is nullptr...aborting";
        RCLCPP_ERROR(this->node_->get_logger(), "%s", err_msg);
        throw ktp::exceptions::DataOmissionException(err_msg);

        return false;
    }
    // ###########################################################################

    // ###########################################################################
    // 태스크 개수 만큼 경로 변환 요청
    // ###########################################################################
    for (auto task_it = this->task_vec_.begin(); task_it != this->task_vec_.end(); ++task_it)
    {
        // 태스크 벡터 인덱스
        const size_t &task_index = std::distance(this->task_vec_.begin(), task_it);

        RCLCPP_INFO(this->node_->get_logger(), "Request Converting Goal to Path task_index : [%zu]", task_index);

        // path_storage 서비스 서버 동작 체크
        const bool &is_service_server_ready = this->path_graph_path_service_client_->wait_for_service(std::chrono::seconds(1));

        if (is_service_server_ready)
        {
            // 서비스 요청
            rclcpp::Client<path_graph_msgs::srv::Path>::FutureAndRequestId future_and_request_id = this->path_graph_path_service_client_->async_send_request(path_request);

            // 서비스 요청 퓨쳐 콜백
            const std::future_status &future_status = future_and_request_id.wait_for(std::chrono::milliseconds(750));

            // 서비스 퓨처 성공 시
            if (future_status == std::future_status::ready)
            {
                RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Path Graph Path Response ---------------------------------------\n");

                // 변환된 경로 할당
                const path_graph_msgs::srv::Path::Response::SharedPtr response = future_and_request_id.future.get();
                const route_msgs::msg::Path &path = response->path;

                const std::string &start_node_type = path.node_list[DEFAULT_INT].type;

                this->node_list_size_ = static_cast<int>(path.node_list.size());
                const std::string &end_node_type = path.node_list[this->node_list_size_ - 1].type;

                RCLCPP_INFO(this->node_->get_logger(), "Converting Goal to Path\n\tstart_node_type : [%s]\n\tend_node_type : [%s]", start_node_type, end_node_type);

                if (start_node_type == NODE_WORKPLACE_NODE_TYPE)
                {
                    RCLCPP_INFO(this->node_->get_logger(), "Current Path is Waiting Area -> Top Chart");
                    this->is_waiting_area_to_top_chart_proceeding = true;
                    this->is_top_chart_to_drop_off_proceeding = false;
                    this->is_drop_off_to_waiting_area_proceeding = false;
                }
                else if(end_node_type == NODE_WORKPLACE_NODE_TYPE)
                {
                    RCLCPP_INFO(this->node_->get_logger(), "Current Path is Drop Off -> Waiting Area");
                    this->is_drop_off_to_waiting_area_proceeding = true;
                    this->is_waiting_area_to_top_chart_proceeding = false;
                    this->is_top_chart_to_drop_off_proceeding = false;
                }
                else
                {
                    RCLCPP_INFO(this->node_->get_logger(), "Current Path is Drop Off -> Waiting Area");
                    this->is_top_chart_to_drop_off_proceeding = true;
                    this->is_waiting_area_to_top_chart_proceeding = false;
                    this->is_drop_off_to_waiting_area_proceeding = false;
                }

                this->path_vec_.push_back(path);

                RCLCPP_INFO(this->node_->get_logger(), "Goal to Path\n\ttask_index : [%zu]\n\tsize : [%zu]", task_index, this->task_vec_.size());

                // 미션 태스크 벡터 요소 개수 만큼 경로 변환 조건
                const bool &is_converting_finished = (static_cast<int>(task_index) == static_cast<int>(this->task_vec_.size() - 1));

                if (is_converting_finished)
                {
                    for (auto path_it = this->path_vec_.begin(); path_it != this->path_vec_.end(); ++path_it)
                    {
                        const size_t &path_index = std::distance(this->path_vec_.begin(), path_it);

                        const route_msgs::msg::Path &task_path = this->path_vec_[path_index];

                        const std::string &path_id = task_path.id;
                        const std::string &path_name = task_path.name;
                        RCLCPP_INFO(this->node_->get_logger(), "path[%zu]\n\tid : [%s]\n\tname : [%s]", path_index, path_id.c_str(), path_name.c_str());

                        const std::vector<route_msgs::msg::Node> &node_list = task_path.node_list;

                        for (auto node_it = node_list.begin(); node_it != node_list.end(); ++node_it)
                        {
                            const size_t &node_index = std::distance(node_list.begin(), node_it);

                            const route_msgs::msg::Node &task_path_node = node_list[node_index];

                            const float &task_longitude = task_path_node.position.longitude;
                            const float &task_latitude = task_path_node.position.latitude;

                            RCLCPP_INFO(this->node_->get_logger(), "node position[%zu]\n\tlongitude : [%f]\n\tlatitude : [%f]", node_index, task_longitude, task_latitude);
                        }
                        RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------\n\n");
                    }
                    return true;
                }
                else
                {
                    continue;
                }
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
    // ###########################################################################
}

path_graph_msgs::srv::Path::Request::SharedPtr ktp::controller::MissionAssigner::build_waiting_area_to_top_chart_path_request()
{
    // ###########################################################################
    // [ 대기 장소 -> 출발지(상차지) ] 경로 조회
    // ###########################################################################
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Waiting Area to Top Chart ---------------------------------------\n");

    path_graph_msgs::srv::Path::Request::SharedPtr waiting_area_to_top_chart_path_request = std::make_shared<path_graph_msgs::srv::Path::Request>();

    // 로봇의 현재 GPS 위치(대기 장소에 있다는 가정)
    const double &waiting_area_longitude = this->ublox_fix_cb_->longitude;
    const double &waiting_area_latitude = this->ublox_fix_cb_->latitude;

    RCLCPP_INFO(this->node_->get_logger(), "Waiting Area Position\n\tlongitude : [%f]\n\tlatitude : [%f]", waiting_area_longitude, waiting_area_latitude);

    route_msgs::msg::Position::UniquePtr waiting_area_position = std::make_unique<route_msgs::msg::Position>();
    waiting_area_position->set__longitude(waiting_area_longitude);
    waiting_area_position->set__latitude(waiting_area_latitude);
    
    const route_msgs::msg::Position &&waiting_area_position_moved = std::move(*(waiting_area_position));
    waiting_area_to_top_chart_path_request->set__position(waiting_area_position_moved);

    // 출발지(상차지) : 미션 태스크의 source 값
    const std::string &top_chart_node = this->task_vec_[DEFAULT_INT].task_data.source;
    RCLCPP_INFO(this->node_->get_logger(), "Waiting Area([%f], [%f]) -> Top Chart([%s])", waiting_area_longitude, waiting_area_latitude, top_chart_node.c_str());
    waiting_area_to_top_chart_path_request->set__start_node("");
    waiting_area_to_top_chart_path_request->set__end_node(top_chart_node);

    RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");

    return waiting_area_to_top_chart_path_request;
    // ###########################################################################
}

path_graph_msgs::srv::Path::Request::SharedPtr ktp::controller::MissionAssigner::build_top_chart_to_drop_off_path_request()
{
    // ###########################################################################
    // ###########################################################################
    // [ 출발지(상차지) -> 하차지 ] 경로 조회
    // ###########################################################################
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Top Chart to Drop Off ---------------------------------------\n");

    path_graph_msgs::srv::Path::Request::SharedPtr top_chart_to_drop_off_path_request = std::make_shared<path_graph_msgs::srv::Path::Request>();

    // 로봇의 현재 GPS 위치(출발지(상차지)에 있다는 가정)
    const double &top_chart_longitude = this->ublox_fix_cb_->longitude;
    const double &top_chart_latitude = this->ublox_fix_cb_->latitude;

    RCLCPP_INFO(this->node_->get_logger(), "Top Chart Position\n\tlongitude : [%f]\n\tlatitude : [%f]", top_chart_longitude, top_chart_latitude);

    route_msgs::msg::Position::UniquePtr top_chart_position = std::make_unique<route_msgs::msg::Position>();
    top_chart_position->set__longitude(top_chart_longitude);
    top_chart_position->set__latitude(top_chart_latitude);

    const route_msgs::msg::Position &&top_chart_to_drop_off_position_moved = std::move(*(top_chart_position));
    top_chart_to_drop_off_path_request->set__position(top_chart_to_drop_off_position_moved);

    // 출발지(상차지) 노드 정보 : 미션 태스크의 source 값
    const std::string &top_chart_node = this->task_vec_[DEFAULT_INT].task_data.source;
    top_chart_to_drop_off_path_request->set__start_node(top_chart_node);

    // 하차지 노드 정보 : 미션 태스크의 goal 값
    const std::string &drop_off_node = this->task_vec_[DEFAULT_INT].task_data.goal[DEFAULT_INT];
    top_chart_to_drop_off_path_request->set__end_node(drop_off_node);

    RCLCPP_INFO(this->node_->get_logger(), "Top Chart([%s]) -> Drop Off([%s]) Path Request", top_chart_node.c_str(), drop_off_node.c_str());

    RCLCPP_INFO(this->node_->get_logger(), "--------------------------------------------------------------------------------------------------\n");

    return top_chart_to_drop_off_path_request;
    // ###########################################################################
}

path_graph_msgs::srv::Path::Request::SharedPtr ktp::controller::MissionAssigner::build_drop_off_to_waiting_area_path_request()
{
    // ###########################################################################
    // ###########################################################################
    // [ 출발지(상차지) -> 하차지 ] 경로 조회
    // ###########################################################################
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------ Drop Off to Waiting Area ---------------------------------------\n");

    path_graph_msgs::srv::Path::Request::SharedPtr drop_off_to_waiting_area_path_request = std::make_shared<path_graph_msgs::srv::Path::Request>();

    // 로봇의 현재 GPS 위치(하차지에 있다는 가정)
    const double &drop_off_longitude = this->ublox_fix_cb_->longitude;
    const double &drop_off_latitude = this->ublox_fix_cb_->latitude;

    RCLCPP_INFO(this->node_->get_logger(), "Drop Off Position\n\tlongitude : [%f]\n\tlatitude : [%f]", drop_off_longitude, drop_off_latitude);

    route_msgs::msg::Position::UniquePtr drop_off_position = std::make_unique<route_msgs::msg::Position>();
    drop_off_position->set__longitude(drop_off_longitude);
    drop_off_position->set__latitude(drop_off_latitude);

    const route_msgs::msg::Position &&drop_off_position_moved = std::move(*(drop_off_position));
    drop_off_to_waiting_area_path_request->set__position(drop_off_position_moved);

    // 하차지 노드 정보 : 미션 태스크의 goal 값
    const std::string &top_chart_node = this->task_vec_[DEFAULT_INT].task_data.goal[DEFAULT_INT];
    drop_off_to_waiting_area_path_request->set__start_node(top_chart_node);
    drop_off_to_waiting_area_path_request->set__end_node("");

    RCLCPP_INFO(this->node_->get_logger(), "-----------------------------------------------------------------------------------------------------\n");

    return drop_off_to_waiting_area_path_request;
    // ###########################################################################
}

void ktp::controller::MissionAssigner::route_to_pose_send_goal()
{
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Route to Pose Send Goal ------------------------");

    const bool &is_path_vec_empty = this->path_vec_.empty();

    // ###########################################################################
    // ###########################################################################
    // 경로 벡터 유효성 검사
    // ###########################################################################
    if (is_path_vec_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose send goal path_vec is empty...aborting");
        return;
    }
    // ###########################################################################

    // ###########################################################################
    // route_to_pose 액션 서버 구동 조회
    // ###########################################################################
    const bool &is_route_to_pose_action_server_ready = this->route_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(1));

    if (!is_route_to_pose_action_server_ready)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "route_to_pose action server is not ready yet...aborting");
        return;
    }
    // ###########################################################################

    route_msgs::action::RouteToPose::Goal::UniquePtr goal = std::make_unique<route_msgs::action::RouteToPose::Goal>();

    // 경로 노드 리스트
    const std::vector<route_msgs::msg::Node> &node_list = this->path_vec_[this->path_current_index_].node_list;

    // ###########################################################################
    // 경로 셋업
    // ###########################################################################
    const int &start_node_index = this->node_current_index_;
    const int &end_node_index = this->node_current_index_ + 1;

    const bool &is_last_node = (start_node_index == (this->node_list_size_ - 1));

    RCLCPP_INFO(
        this->node_->get_logger(),
        "Goal Node List Info\n\tnode_list_size : [%d]\n\tstart_node_index : [%d]\n\tend_node_index : [%d]\n\tis_last_node? : [%d]",
        this->node_list_size_,
        start_node_index,
        end_node_index,
        is_last_node);

    if (is_last_node)
    {
        RCLCPP_WARN(this->node_->get_logger(), "This Goal is Last Index Node : [%d]", this->node_current_index_);
        return;
    }

    const route_msgs::msg::Node &start_node = node_list[start_node_index];
    goal->set__start_node(start_node);

    const route_msgs::msg::Node &end_node = node_list[end_node_index];
    goal->set__end_node(end_node);
    // ###########################################################################

    rclcpp_action::Client<route_msgs::action::RouteToPose>::SendGoalOptions goal_opts = rclcpp_action::Client<route_msgs::action::RouteToPose>::SendGoalOptions();
    goal_opts.goal_response_callback = std::bind(&ktp::controller::MissionAssigner::route_to_pose_goal_response_cb, this, _1);
    goal_opts.feedback_callback = std::bind(&ktp::controller::MissionAssigner::route_to_pose_feedback_cb, this, _1, _2);
    goal_opts.result_callback = std::bind(&ktp::controller::MissionAssigner::route_to_pose_result_cb, this, _1);

    const route_msgs::action::RouteToPose::Goal &&goal_moved = std::move(*(goal));

    std::shared_future<rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr> goal_future = this->route_to_pose_action_client_->async_send_goal(goal_moved, goal_opts);
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>> &goal_handle = goal_future.get();

    RCLCPP_INFO(
        this->node_->get_logger(),
        "route_to_pose goal sent\n\tis_feedback_aware : [%d]\n\tis_result_aware : [%d]",
        goal_handle->is_feedback_aware(),
        goal_handle->is_result_aware());

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
        RCLCPP_INFO(this->node_->get_logger(), "goal handle accepted!");
    }

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}

void ktp::controller::MissionAssigner::route_to_pose_feedback_cb(rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr goal_handle, const std::shared_ptr<const route_msgs::action::RouteToPose::Feedback> feedback)
{
    (void)goal_handle;

    const int32_t &status_code = feedback->status_code;

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "----------------------- Route to Pose Feedback -------------------------");

    RCLCPP_INFO(this->node_->get_logger(), "status_code : [%d]", status_code);

    switch (status_code)
    {
    case ROUTE_TO_POSE_FEEDBACK_NAVIGATION_STARTED_CODE:
    {
        if (this->is_waiting_area_to_top_chart_proceeding)
        {
            RCLCPP_INFO(this->node_->get_logger(), "Waiting Area -> Top Chart Started, task_current_index : [%d]", this->task_current_index_);
            this->mission_notificator_->notify_mission_status(MISSION_TASK_STARTED_CODE, this->task_vec_[this->task_current_index_], this->task_current_index_);
        }
        else if (this->is_top_chart_to_drop_off_proceeding)
        {
            RCLCPP_INFO(this->node_->get_logger(), "Top Chart -> Drop Off Started, task_current_index : [%d]", this->task_current_index_);
            this->mission_notificator_->notify_mission_status(MISSION_TASK_ON_PROGRESS_CODE, this->task_vec_[this->task_current_index_], this->task_current_index_);
        }
        break;
    }
    default:
        RCLCPP_ERROR(this->node_->get_logger(), "unkwon feedback status_code : [%d]", status_code);
        break;
    }

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}

void ktp::controller::MissionAssigner::route_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::WrappedResult &wrapped_result)
{
    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    RCLCPP_INFO(this->node_->get_logger(), "------------------------ Route to Pose Result --------------------------");

    switch (wrapped_result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            this->node_current_index_++;
            const bool is_last_node_index = (this->node_current_index_ == (this->node_list_size_ - 1));

            RCLCPP_INFO(
                    this->node_->get_logger(),
                    "Goal was succeeded\n\tnode_current_index : [%d]\n\tis_last_node_index : [%d]",
                    this->node_current_index_,
                    is_last_node_index);

            if (is_last_node_index)
            {
                if (this->is_waiting_area_to_top_chart_proceeding)
                {
                    RCLCPP_INFO(this->node_->get_logger(), "Source Arrived, task_current_index : [%d]", this->task_current_index_);
                    this->mission_notificator_->notify_mission_status(MISSION_TASK_SOURCE_ARRIVED_CODE,
                                                                      this->task_vec_[this->task_current_index_],
                                                                      this->task_current_index_);

                    this->node_current_index_ = DEFAULT_INT;
                    this->node_list_size_ = DEFAULT_INT;

                    this->path_vec_.clear();
                    this->path_current_index_ = DEFAULT_INT;
                    this->path_vec_size_ = DEFAULT_INT;
                    this->is_waiting_area_to_top_chart_proceeding = false;

                    // ###########################################################################
                    // ###########################################################################
                    // [ 출발지(상차지) -> 하차지 ] 경로 조회
                    // ###########################################################################
                    const path_graph_msgs::srv::Path::Request::SharedPtr &top_chart_to_drop_off_path_request = this->build_top_chart_to_drop_off_path_request();
                    const bool &is_path_request_succeeded = this->request_converting_goal_to_path(top_chart_to_drop_off_path_request);

                    if (is_path_request_succeeded)
                    {
                        RCLCPP_INFO(this->node_->get_logger(), "Due to Source Arrived, Ready to Send Goal Top Chart -> Drop Off");
                    }
                    // ###########################################################################
                }
                else if (this->is_top_chart_to_drop_off_proceeding)
                {
                    RCLCPP_INFO(this->node_->get_logger(), "Source Arrived, task_current_index : [%d]", this->task_current_index_);
                    this->mission_notificator_->notify_mission_status(MISSION_TASK_DEST_ARRIVED_CODE,
                                                                      this->task_vec_[this->task_current_index_],
                                                                      this->task_current_index_);

                    this->node_current_index_ = DEFAULT_INT;
                    this->node_list_size_ = DEFAULT_INT;

                    this->path_vec_.clear();
                    this->path_current_index_ = DEFAULT_INT;
                    this->path_vec_size_ = DEFAULT_INT;
                    this->is_waiting_area_to_top_chart_proceeding = false;

                    // ###########################################################################
                    // ###########################################################################
                    // [ 하차지 -> 대기 장소 ] 경로 조회
                    // ###########################################################################
                    const path_graph_msgs::srv::Path::Request::SharedPtr &drop_off_to_waiting_area_path_request = this->build_drop_off_to_waiting_area_path_request();
                    const bool &is_path_request_succeeded = this->request_converting_goal_to_path(drop_off_to_waiting_area_path_request);

                    if (is_path_request_succeeded)
                    {
                        RCLCPP_INFO(this->node_->get_logger(), "Due to Dest Arrived, Ready to Send Goal Drop Off -> Waiting Area");
                    }
                    // ###########################################################################
                }
                else if (this->is_drop_off_to_waiting_area_proceeding)
                {
                    return;
                }
            }
            else
            {
                this->route_to_pose_send_goal();
            }
            break;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->node_->get_logger(), "Goal was aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->node_->get_logger(), "Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(this->node_->get_logger(), "Unknown result code");
        break;
    }

    RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}