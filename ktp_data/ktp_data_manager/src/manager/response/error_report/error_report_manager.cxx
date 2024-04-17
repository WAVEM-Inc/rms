#include "manager/response/error_report/error_report_manager.hxx"

ktp::data::ErrorReportManager::ErrorReportManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->error_reception_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions error_reception_subscription_opts;
    error_reception_subscription_opts.callback_group = this->error_reception_subscription_cb_group_;
    this->error_reception_subscription_ = this->node_->create_subscription<std_msgs::msg::String>(
        ERROR_RECEPTION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::ErrorReportManager::error_reception_subscription_cb, this, _1),
        error_reception_subscription_opts);

    this->error_report_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions error_report_publisher_opts;
    error_report_publisher_opts.callback_group = this->error_report_to_itf_publisher_cb_group_;
    this->error_report_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ErrorReport>(
        ERROR_REPORT_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        error_report_publisher_opts);
    
    RCLCPP_INFO(this->node_->get_logger(), "=============== Response ErrorReportManager initialized ===============");
}

ktp::data::ErrorReportManager::~ErrorReportManager()
{
}

void ktp::data::ErrorReportManager::error_reception_subscription_cb(const std_msgs::msg::String::SharedPtr error_reception_cb)
{
    const char *error_msg = error_reception_cb->data.c_str();

    RCLCPP_INFO(this->node_->get_logger(), "Error Report error_msgs : [%s]", error_msg);

    ktp_data_msgs::msg::ErrorReport::UniquePtr error_report = std::make_unique<ktp_data_msgs::msg::ErrorReport>();
    
    const std::string &create_time = get_current_time();
    error_report->set__create_time(create_time);
    error_report->set__error_code(error_msg);

    const ktp_data_msgs::msg::ErrorReport &&error_report_moved = std::move(*(error_report));

    this->error_report_to_itf_publisher_->publish(error_report_moved);
}