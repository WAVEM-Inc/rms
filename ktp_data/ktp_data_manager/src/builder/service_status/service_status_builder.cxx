#include "builder/service_status/service_status_builder.hxx"

ktp::build::ServiceStatusBuilder::ServiceStatusBuilder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
}

ktp::build::ServiceStatusBuilder::~ServiceStatusBuilder()
{
}

ktp_data_msgs::msg::ServiceStatusTaskData ktp::build::ServiceStatusBuilder::build_rbt_service_status_task_data()
{
    ktp_data_msgs::msg::ServiceStatusTaskData::UniquePtr rbt_service_status_task_data = std::make_unique<ktp_data_msgs::msg::ServiceStatusTaskData>();

    // ############################################
    // map_id
    // 목적지 맵 아이디
    rbt_service_status_task_data->set__map_id("asdf");
    // ############################################
    
    // ############################################
    // goal_vec
    // 목적니 노드 리스트 (순회, 경유지 존재할 경우 여러 개, 그 외 1개)
    std::vector<std::string> goal_vec;
    goal_vec.push_back("asdfsadf");
    rbt_service_status_task_data->set__goal(goal_vec);
    // ############################################

    // ############################################
    // source
    // 출발지 노드
    rbt_service_status_task_data->set__source("qwer");
    // ############################################

    const ktp_data_msgs::msg::ServiceStatusTaskData &&rbt_service_status_task_data_moved = std::move(*rbt_service_status_task_data);

    return rbt_service_status_task_data_moved;
}

std::vector<ktp_data_msgs::msg::ServiceStatusTask> ktp::build::ServiceStatusBuilder::build_rbt_service_status_task()
{
    ktp_data_msgs::msg::ServiceStatusTask::UniquePtr rbt_service_status_task = std::make_unique<ktp_data_msgs::msg::ServiceStatusTask>();

    // ############################################
    // task_id
    // 미션 상세 태스크 아이디 : mission_id + seq
    rbt_service_status_task->set__task_id("ajskakshaksadkfdsasdg");
    // ############################################

    // ############################################
    // task_code
    // 태스트 코드
    rbt_service_status_task->set__task_code(ktp::enums::TaskCodeMap.at(ktp::enums::TaskCode::RETURNING));
    // ############################################

    // ############################################
    // status
    // 태스트 진행 상태
    rbt_service_status_task->set__status(ktp::enums::StatusMap.at(ktp::enums::Status::STARTED));
    // ############################################

    // ############################################
    // seq
    // 태스트 순서 (0부터)
    rbt_service_status_task->set__seq(1);
    // ############################################

    // ############################################
    // task_data
    // 태스트 상세 데이터
    rbt_service_status_task->set__task_data(this->build_rbt_service_status_task_data());
    // ############################################

    const ktp_data_msgs::msg::ServiceStatusTask &&rbt_service_status_task_moved = std::move(*rbt_service_status_task);

    std::vector<ktp_data_msgs::msg::ServiceStatusTask> rbt_service_status_task_vec;
    rbt_service_status_task_vec.push_back(rbt_service_status_task_moved);

    return rbt_service_status_task_vec;
}

ktp_data_msgs::msg::ServiceStatus ktp::build::ServiceStatusBuilder::build_rbt_service_status()
{
    ktp_data_msgs::msg::ServiceStatus::UniquePtr rbt_service_status = std::make_unique<ktp_data_msgs::msg::ServiceStatus>();

    // ############################################
    // create_time
    // 생성 일시
    const std::string &create_time = get_current_time();
    rbt_service_status->set__create_time(create_time);
    // ############################################

    // ############################################
    // mission_code
    // 미션 코드
    rbt_service_status->set__mission_code("ktdelivery");
    // ############################################

    // ############################################
    // mission_id
    // 임무 아이디 : deviceId + create_time
    const char *device_id = "aslkhgiqwkg";
    std::string mission_id = device_id + create_time;
    rbt_service_status->set__mission_id(mission_id);
    // ############################################

    const ktp_data_msgs::msg::ServiceStatus &&rbt_service_status_moved = std::move(*rbt_service_status);

    return rbt_service_status_moved;
}