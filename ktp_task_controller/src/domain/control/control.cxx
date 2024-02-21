#include "domain/control/control.hxx"

ktp::domain::Control::Control()
{
}

ktp::domain::Control::~Control()
{
}

ktp_data_msgs::msg::Control ktp::domain::Control::get__control()
{
    return this->control_;
}

void ktp::domain::Control::set__control(ktp_data_msgs::msg::Control control)
{
    this->control_ = control;
}