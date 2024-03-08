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