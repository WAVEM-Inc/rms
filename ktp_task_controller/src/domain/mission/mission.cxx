#include "domain/mission/mission.hxx"

ktp::domain::Mission::Mission()
{
}

ktp::domain::Mission::~Mission()
{
}

uint8_t ktp::domain::Mission::get__response_code()
{
    return this->response_code_;
}

void ktp::domain::Mission::set__response_code(uint8_t response_code)
{
    this->response_code_ = response_code;
}

uint8_t ktp::domain::Mission::get__status_code()
{
    return this->status_code_;
}

void ktp::domain::Mission::set__status_code(uint8_t mission_status_code)
{
    this->status_code_ = mission_status_code;
}

ktp_data_msgs::msg::Mission ktp::domain::Mission::get__mission()
{
    return this->mission_;
}

void ktp::domain::Mission::set__mission(ktp_data_msgs::msg::Mission mission)
{
    this->mission_ = mission;
}

ktp::domain::NavigationStatus::NavigationStatus()
    : drive_status_(NAVIGATION_STATUS_WAITING_CODE)
{
}

ktp::domain::NavigationStatus::~NavigationStatus()
{
}

int ktp::domain::NavigationStatus::get__drive_status()
{
    return this->drive_status_;
}

void ktp::domain::NavigationStatus::set__drive_status(int drive_status)
{
    this->drive_status_ = drive_status;
}

route_msgs::msg::Node ktp::domain::NavigationStatus::get__start_node()
{
    return this->start_node_;
}

void ktp::domain::NavigationStatus::set__start_node(route_msgs::msg::Node start_node)
{
    this->start_node_ = start_node;
}

route_msgs::msg::Node ktp::domain::NavigationStatus::get__end_node()
{
    return this->end_node_;
}

void ktp::domain::NavigationStatus::set__end_node(route_msgs::msg::Node end_node)
{
    this->end_node_ = end_node;
}

ktp_data_msgs::msg::MissionTask ktp::domain::NavigationStatus::get__mission_task()
{
    return this->mission_task_;
}

void ktp::domain::NavigationStatus::set__mission_task(ktp_data_msgs::msg::MissionTask mission_task)
{
    this->mission_task_ = mission_task;
}