#include "builder/builder.hxx"

ktp::data::Builder::Builder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->battery_state_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions battery_state_subscription_opts;
    battery_state_subscription_opts.callback_group = this->battery_state_subscription_cb_group_;
    this->battery_state_subscription_ = this->node_->create_subscription<sensor_msgs::msg::BatteryState>(
        BATTERY_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::Builder::battery_state_subscription_cb, this, _1),
        battery_state_subscription_opts);

    this->gps_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;
    this->gps_subscription_ = this->node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        GPS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::Builder::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->rtt_odom_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rtt_odom_subscription_opts;
    rtt_odom_subscription_opts.callback_group = this->rtt_odom_subscription_cb_group_;
    this->rtt_odom_subscription_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        RTT_ODOM_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::Builder::rtt_odom_subscription_cb, this, _1),
        rtt_odom_subscription_opts);
}

ktp::data::Builder::~Builder()
{
}

std::string ktp::data::Builder::get_current_time()
{
    const std::chrono::_V2::system_clock::time_point &now = std::chrono::system_clock::now();
    const std::time_t &time = std::chrono::system_clock::to_time_t(now);
    const std::tm &tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%y%m%d%H%M%S") << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    return oss.str();
}

void ktp::data::Builder::battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb)
{
    this->battery_state_cb_ = battery_state_cb;
}

void ktp::data::Builder::gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_cb)
{
    this->gps_cb_ = gps_cb;
}

void ktp::data::Builder::rtt_odom_subscription_cb(const geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb)
{
    this->rtt_odom_cb_ = rtt_odom_cb;
}

ktp_data_msgs::msg::Status ktp::data::Builder::build_rbt_status()
{
    ktp_data_msgs::msg::Status::UniquePtr rbt_status = std::make_unique<ktp_data_msgs::msg::Status>();

    double battery = 0.0;

    if (this->battery_state_cb_ != nullptr)
    {
        battery = this->battery_state_cb_->percentage;
    }
    rbt_status->set__battery(battery);

    double longitude = 0.0;
    double latitude = 0.0;

    if (this->gps_cb_ != nullptr)
    {
        longitude = this->gps_cb_->longitude;
        latitude = this->gps_cb_->latitude;
    }
    rbt_status->set__x(longitude);
    rbt_status->set__y(latitude);

    double heading = 0.0;

    if (this->rtt_odom_cb_ != nullptr)
    {
        heading = this->rtt_odom_cb_->pose.orientation.y;
    }
    rbt_status->set__heading(heading);

    const std::string &create_time = this->get_current_time();
    rbt_status->set__create_time(create_time);

    RCLCPP_INFO(
        this->node_->get_logger(),
        "Builder RbtStatus\n\tbattery : [%f]\n\tcreate_time : [%s]\n\tlongitude : [%f]\n\tlatitude : [%f]\n\theading : [%f]",
        battery,
        create_time.c_str(),
        longitude,
        latitude,
        heading);

    const ktp_data_msgs::msg::Status &&rbt_status_moved = std::move(*rbt_status);

    return rbt_status_moved;
}