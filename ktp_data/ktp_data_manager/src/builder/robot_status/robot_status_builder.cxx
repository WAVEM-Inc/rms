#include "builder/robot_status/robot_status_builder.hxx"

ktp::build::RobotStatusBuilder::RobotStatusBuilder(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->battery_state_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions battery_state_subscription_opts;
    battery_state_subscription_opts.callback_group = this->battery_state_subscription_cb_group_;
    this->battery_state_subscription_ = this->node_->create_subscription<sensor_msgs::msg::BatteryState>(
        BATTERY_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::build::RobotStatusBuilder::battery_state_subscription_cb, this, _1),
        battery_state_subscription_opts);

    this->gps_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;
    this->gps_subscription_ = this->node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        GPS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::build::RobotStatusBuilder::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->rtt_odom_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rtt_odom_subscription_opts;
    rtt_odom_subscription_opts.callback_group = this->rtt_odom_subscription_cb_group_;
    this->rtt_odom_subscription_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        RTT_ODOM_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::build::RobotStatusBuilder::rtt_odom_subscription_cb, this, _1),
        rtt_odom_subscription_opts);
}

ktp::build::RobotStatusBuilder::~RobotStatusBuilder()
{
}

void ktp::build::RobotStatusBuilder::battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb)
{
    this->battery_state_cb_ = battery_state_cb;

    double battery = 0.0;
    if (this->battery_state_cb_ != nullptr)
    {
        battery = this->battery_state_cb_->percentage;

        if (battery < 0 || battery > 100)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Builder RbtStatus battery is lower than 0 or over than 100...");
            this->battery_cb_flag_ = false;
            return;
        }
        else
        {
            this->battery_cb_flag_ = true;
        }
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Builder RbtStatus battery_cb is nullptr...");
        this->battery_cb_flag_ = false;
        return;
    }
}

void ktp::build::RobotStatusBuilder::gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_cb)
{
    this->gps_cb_ = gps_cb;

    if (this->gps_cb_ != nullptr)
    {
        this->gps_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Builder RbtStatus gps_cb is nullptr...");
        this->gps_cb_flag_ = false;
        return;
    }
}

void ktp::build::RobotStatusBuilder::rtt_odom_subscription_cb(const geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb)
{
    this->rtt_odom_cb_ = rtt_odom_cb;

    if (this->rtt_odom_cb_ != nullptr)
    {
        this->rtt_odom_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "Builder RbtStatus rtt_odom_cb is nullptr...");
        this->rtt_odom_cb_flag_ = false;
        return;
    }
}

ktp_data_msgs::msg::StatusService ktp::build::RobotStatusBuilder::build_service()
{
    ktp_data_msgs::msg::StatusService::UniquePtr rbt_status_service = std::make_unique<ktp_data_msgs::msg::StatusService>();

    ktp_data_msgs::msg::StatusServiceEnv::UniquePtr rbt_status_service_env = std::make_unique<ktp_data_msgs::msg::StatusServiceEnv>();

    // ############################################
    // temperature
    // 온도 (단위 : 섭씨)
    rbt_status_service_env->set__temperature(DEFAULT_DOUBLE);

    // ############################################
    // humidity
    // 습도 (단위 : percent)
    rbt_status_service_env->set__humidity(DEFAULT_DOUBLE);
    // ############################################

    const ktp_data_msgs::msg::StatusServiceEnv &&rbt_status_service_env_moved = std::move(*rbt_status_service_env);

    rbt_status_service->set__env(rbt_status_service_env_moved);
    const ktp_data_msgs::msg::StatusService &&rbt_status_service_moved = std::move(*rbt_status_service);
    
    return rbt_status_service_moved;
}

ktp_data_msgs::msg::Status ktp::build::RobotStatusBuilder::build_rbt_status()
{
    ktp_data_msgs::msg::Status::UniquePtr rbt_status = std::make_unique<ktp_data_msgs::msg::Status>();

    // ############################################
    // map_id
    // 현재 맵 id
    const char *map_id = "";
    rbt_status->set__map_id(map_id);
    // ############################################

    // ############################################
    // battery
    // 배터리 잔여율(%)
    RCLCPP_INFO(this->node_->get_logger(), "Builder RbtStatus battery_cb_flag : [%d]", this->battery_cb_flag_);
    if (this->battery_cb_flag_ == true)
    {
        rbt_status->set__battery(this->battery_state_cb_->percentage);
    }
    else
    {
        rbt_status->set__battery(DEFAULT_DOUBLE);
    }
    // ############################################

    // ############################################
    // drive_status
    // 로봇의 주행 상태
    rbt_status->set__drive_status(ktp::enums::DriveStatus::DRIVING_NORMALLY);
    // ############################################

    // ############################################
    // create_time
    // 생성 일시
    const std::string &create_time = get_current_time();
    rbt_status->set__create_time(create_time);
    // ############################################

    // ############################################
    // speed
    // 로봇의 주행 속도
    rbt_status->set__speed(DEFAULT_DOUBLE);
    // ############################################

    // ############################################
    // from_node
    // 현재 출발 노드
    rbt_status->set__from_node("qwlbgasugbqwqjkgbsadjkgbadgjsqa");
    // ############################################

    // ############################################
    // floor
    // 로봇이 위치한 층
    rbt_status->set__floor("1F");
    // ############################################

    // ############################################
    // section
    // 로봇이 위치한 구역
    rbt_status->set__section("iqwgrjkbajwdsbguqwbg;jaksddsfadgq");
    // ############################################

    // ############################################
    // to_node
    // 현재 목적 노드
    rbt_status->set__to_node("aikweqjkbhgsaoujdbguoqwg");
    // ############################################

    // ############################################
    // charge
    // 로봇의 충전 상태
    rbt_status->set__charge(false);
    // ############################################

    // ############################################
    // x, y
    // x(longitude), y(latitude) 좌표
    RCLCPP_INFO(this->node_->get_logger(), "Builder RbtStatus gps_cb_flag : [%d]", this->gps_cb_flag_);

    double longitude = 0.0;
    double latitude = 0.0;

    if (this->gps_cb_flag_ == true)
    {
        longitude = this->gps_cb_->longitude;
        latitude = this->gps_cb_->latitude;
    }
    rbt_status->set__x(longitude);
    rbt_status->set__y(latitude);
    // ############################################

    // ############################################
    // heading
    // 로봇의 방향 값(degree 기준)
    RCLCPP_INFO(this->node_->get_logger(), "Builder RbtStatus rtt_odom_cb_flag : [%d]", this->rtt_odom_cb_flag_);

    double heading = 0.0;
    if (this->rtt_odom_cb_flag_ == true)
    {
        heading = this->rtt_odom_cb_->pose.orientation.y;
    }
    rbt_status->set__heading(heading);
    // ############################################

    // ############################################
    // is_indoor
    // 실내/실외 구분값
    rbt_status->set__is_indoor(true);
    // ############################################

    // ############################################
    // coord_code
    // 로봇 주행 좌표계 코드
    rbt_status->set__coord_code("WGS84");
    // ############################################

    // ############################################
    // service_mode
    // 서비스 모드
    rbt_status->set__service_mode("delivering");
    // ############################################

    // ############################################
    // service
    // 서비스 특화 데이터
    const ktp_data_msgs::msg::StatusService &rbt_status_service = this->build_service();
    rbt_status->set__service(rbt_status_service);
    // ############################################

    // ############################################
    // firmware_version
    rbt_status->set__firmware_version("humble");
    // ############################################

    RCLCPP_INFO(
        this->node_->get_logger(),
        "Builder RbtStatus\n\tbattery : [%f]\n\tcreate_time : [%s]\n\tlongitude : [%f]\n\tlatitude : [%f]\n\theading : [%f]",
        rbt_status->battery,
        create_time.c_str(),
        longitude,
        latitude,
        heading);

    const ktp_data_msgs::msg::Status &&rbt_status_moved = std::move(*rbt_status);

    return rbt_status_moved;
}