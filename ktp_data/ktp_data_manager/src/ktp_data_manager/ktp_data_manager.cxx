#include "ktp_data_manager/ktp_data_manager.hxx"

ktp::data::Manager::Manager()
    : Node(NODE_NAME)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] node has been created", NODE_NAME);
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(NODE_NAME, "failed to create %s node", NODE_NAME);
    }

    this->rbt_status_publisher_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->rbt_status_publisher_timer_ = this->node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ktp::data::Manager::rbt_status_publisher_timer_cb, this, _1),
        this->rbt_status_publisher_timer_cb_group_);

    this->rbt_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_status_publisher_opts;
    rbt_status_publisher_opts.callback_group = this->rbt_status_publisher_cb_group_;
    this->rbt_status_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::Status>(
        RBT_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_status_publisher_opts);

    this->rbt_service_status_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_service_status_publisher_opts;
    rbt_service_status_publisher_opts.callback_group = this->rbt_service_status_publisher_cb_group_;
    this->rbt_service_status_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ServiceStatus>(
        RBT_SERVICE_STATUS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_service_status_publisher_opts);

    this->rbt_error_report_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_error_report_publisher_opts;
    rbt_error_report_publisher_opts.callback_group = this->rbt_error_report_publisher_cb_group_;
    this->rbt_error_report_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ErrorReport>(
        RBT_ERROR_REPORT_TOIPC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_error_report_publisher_opts
    );

    this->rbt_control_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_control_publisher_opts;
    rbt_control_publisher_opts.callback_group = this->rbt_control_publisher_cb_group_;
    this->rbt_control_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::Control>(
        RBT_CONTROL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_control_publisher_opts
    );

    this->rbt_control_report_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_control_report_publisher_opts;
    rbt_control_report_publisher_opts.callback_group = this->rbt_control_report_publisher_cb_group_;
    this->rbt_control_report_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::ControlReport>(
        RBT_CONTROL_REPORT_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_control_report_publisher_opts
    );

    this->rbt_mission_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_mission_publisher_opts;
    rbt_mission_publisher_opts.callback_group = this->rbt_mission_publisher_cb_group_;
    this->rbt_mission_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::Mission>(
        RBT_MISSION_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_mission_publisher_opts
    );

    this->rbt_graph_list_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions rbt_graph_list_publisher_opts;
    rbt_graph_list_publisher_opts.callback_group = this->rbt_graph_list_publisher_cb_group_;
    this->rbt_graph_list_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::GraphList>(
        RBT_GRAPH_LIST_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        rbt_graph_list_publisher_opts
    );
}

ktp::data::Manager::~Manager()
{
}

void ktp::data::Manager::rbt_status_publisher_timer_cb()
{
    
}