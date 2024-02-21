#include "domain/mission/mission.hxx"

ktp::domain::Mission::Mission()
{
}

ktp::domain::Mission::~Mission()
{
}

ktp_data_msgs::msg::Mission ktp::domain::Mission::get__mission()
{
    return this->mission_;
}

void ktp::domain::Mission::set__mission(ktp_data_msgs::msg::Mission mission)
{
    this->mission_ = mission;
}