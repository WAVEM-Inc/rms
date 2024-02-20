#include "builder/builder.hxx"

ktp::build::MainBuilder::MainBuilder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->robot_status_builder_ = std::make_shared<ktp::build::RobotStatusBuilder>(this->node_);
}

ktp::build::MainBuilder::~MainBuilder()
{
}

ktp_data_msgs::msg::Status ktp::build::MainBuilder::build_rbt_status()
{
    const ktp_data_msgs::msg::Status &rbt_status = this->robot_status_builder_->build_rbt_status();
    return rbt_status;
}

ktp_data_msgs::msg::ServiceStatus ktp::build::MainBuilder::build_rbt_service_status()
{

}