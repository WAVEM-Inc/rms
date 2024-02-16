#include "resource_model/resource_model.hxx"

ktp::data::ResourceModel::ResourceModel()
{
    this->rbt_status_ = std::make_shared<ktp_data_msgs::msg::Status>();
    this->rbt_service_status_ = std::make_shared<ktp_data_msgs::msg::ServiceStatus>();
    this->rbt_error_report_ = std::make_shared<ktp_data_msgs::msg::ErrorReport>();
    this->rbt_control_ = std::make_shared<ktp_data_msgs::msg::Control>();
    this->rbt_control_report_ = std::make_shared<ktp_data_msgs::msg::ControlReport>();
    this->rbt_mission_ = std::make_shared<ktp_data_msgs::msg::Mission>();
    this->rbt_graph_list_ = std::make_shared<ktp_data_msgs::msg::GraphList>();
}

ktp::data::ResourceModel::~ResourceModel()
{
}

ktp_data_msgs::msg::Status ktp::data::ResourceModel::get__rbt_status()
{
    return *(this->rbt_status_);
}

void ktp::data::ResourceModel::set__rbt_status(ktp_data_msgs::msg::Status::SharedPtr rbt_status)
{
    this->rbt_status_ = rbt_status;
}

ktp_data_msgs::msg::ServiceStatus ktp::data::ResourceModel::get__rbt_service_status()
{
    return *(this->rbt_service_status_);
}

ktp_data_msgs::msg::ErrorReport ktp::data::ResourceModel::get__rbt_error_report()
{
    return *(this->rbt_error_report_);
}

ktp_data_msgs::msg::Control ktp::data::ResourceModel::get__rbt_control()
{
    return *(this->rbt_control_);
}

ktp_data_msgs::msg::ControlReport ktp::data::ResourceModel::get__rbt_control_report()
{
    return *(this->rbt_control_report_);
}

ktp_data_msgs::msg::Mission ktp::data::ResourceModel::get__rbt_mission()
{
    return *(this->rbt_mission_);
}

ktp_data_msgs::msg::GraphList ktp::data::ResourceModel::get__rbt_graph_list()
{
    return *(this->rbt_graph_list_);
}