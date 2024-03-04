#include "domain/mission/mission.hxx"

ktp::domain::Mission::Mission()
{
}

ktp::domain::Mission::~Mission()
{
}

u_int16_t ktp::domain::Mission::get__mission_status_code()
{
    return this->mission_status_code_;
}

void ktp::domain::Mission::set__mission_status_code(u_int16_t mission_status_code)
{
    this->mission_status_code_ = mission_status_code;
}

ktp_data_msgs::msg::Mission ktp::domain::Mission::get__mission()
{
    return this->mission_;
}

void ktp::domain::Mission::set__mission(ktp_data_msgs::msg::Mission mission)
{
    this->mission_ = mission;
}