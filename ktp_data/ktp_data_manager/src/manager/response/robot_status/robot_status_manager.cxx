#include "manager/response/robot_status/robot_status_manager.hxx"

ktp::data::RobotStatusManager::RobotStatusManager(const rclcpp::Node::SharedPtr node)
    : node_(node)
{
    this->robot_status_to_itf_publisher_timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->robot_status_to_itf_publisher_timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(STATUS_TO_ITF_RATE),
        std::bind(&ktp::data::RobotStatusManager::robot_status_publisher_timer_cb, this),
        this->robot_status_to_itf_publisher_timer_cb_group_);

    this->robot_status_to_itf_publisher_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions robot_status_publisher_opts;
    robot_status_publisher_opts.callback_group = this->robot_status_to_itf_publisher_cb_group_;
    this->robot_status_to_itf_publisher_ = this->node_->create_publisher<ktp_data_msgs::msg::Status>(
        STATUS_TO_ITF_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        robot_status_publisher_opts);

    this->battery_state_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions battery_state_subscription_opts;
    battery_state_subscription_opts.callback_group = this->battery_state_subscription_cb_group_;
    this->battery_state_subscription_ = this->node_->create_subscription<sensor_msgs::msg::BatteryState>(
        BATTERY_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::battery_state_subscription_cb, this, _1),
        battery_state_subscription_opts);

    this->velocity_state_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions velocity_state_subscription_opts;
    velocity_state_subscription_opts.callback_group = this->velocity_state_subscription_cb_group_;
    this->velocity_state_subscription_ = this->node_->create_subscription<robot_status_msgs::msg::VelocityStatus>(
        VELOCITY_STATE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::velocity_state_subscription_cb, this, _1),
        velocity_state_subscription_opts);

    this->gps_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions gps_subscription_opts;
    gps_subscription_opts.callback_group = this->gps_subscription_cb_group_;
    this->gps_subscription_ = this->node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        GPS_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::gps_subscription_cb, this, _1),
        gps_subscription_opts);

    this->rtt_odom_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions rtt_odom_subscription_opts;
    rtt_odom_subscription_opts.callback_group = this->rtt_odom_subscription_cb_group_;
    this->rtt_odom_subscription_ = this->node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        RTT_ODOM_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::rtt_odom_subscription_cb, this, _1),
        rtt_odom_subscription_opts);

    this->temperature_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions temperature_subscription_opts;
    temperature_subscription_opts.callback_group = this->temperature_subscription_cb_group_;
    this->temperature_subscription_ = this->node_->create_subscription<sensor_msgs::msg::Temperature>(
        TEMPERATURE_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::temperature_subscription_cb, this, _1),
        temperature_subscription_opts);

    this->humidity_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions humidity_subscription_opts;
    humidity_subscription_opts.callback_group = this->humidity_subscription_cb_group_;
    this->humidity_subscription_ = this->node_->create_subscription<sensor_msgs::msg::RelativeHumidity>(
        HUMIDITY_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::humidity_subscription_cb, this, _1),
        humidity_subscription_opts);

    this->robot_navigation_status_from_task_ctrl_subscription_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions robot_navigation_status_from_task_ctrl_subscription_opts;
    robot_navigation_status_from_task_ctrl_subscription_opts.callback_group = this->robot_navigation_status_from_task_ctrl_subscription_cb_group_;
    this->robot_navigation_status_from_task_ctrl_subscription_ = this->node_->create_subscription<ktp_data_msgs::msg::Status>(
        ROBOT_NAVIGATION_STATUS_FROM_TASK_CTRL_TOPIC,
        rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)),
        std::bind(&ktp::data::RobotStatusManager::robot_navigation_status_from_task_ctrl_cb, this, _1),
        robot_navigation_status_from_task_ctrl_subscription_opts);

    RCLCPP_INFO(this->node_->get_logger(), "=============== Response RbtStatusManager initialized ===============");
}

ktp::data::RobotStatusManager::~RobotStatusManager()
{
}

void ktp::data::RobotStatusManager::robot_status_publisher_timer_cb()
{
    // RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
    // RCLCPP_INFO(this->node_->get_logger(), "------------------------ Robot Status Publish --------------------------");
    const ktp_data_msgs::msg::Status &robot_status = this->build_robot_status();
    this->robot_status_to_itf_publisher_->publish(robot_status);
    // RCLCPP_INFO(this->node_->get_logger(), "------------------------------------------------------------------------");
}

void ktp::data::RobotStatusManager::battery_state_subscription_cb(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_cb)
{
    this->battery_state_cb_ = battery_state_cb;

    double battery = 0.0;
    if (this->battery_state_cb_ != nullptr)
    {
        battery = this->battery_state_cb_->voltage;

        if (battery < 0 || battery > 100)
        {
            RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager battery is lower than 0 or over than 100...");
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
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager battery_cb is nullptr...");
        this->battery_cb_flag_ = false;
        return;
    }
}

void ktp::data::RobotStatusManager::velocity_state_subscription_cb(const robot_status_msgs::msg::VelocityStatus::SharedPtr velocity_state_cb)
{
    this->velocity_state_cb_ = velocity_state_cb;

    if (this->velocity_state_cb_ != nullptr)
    {
        this->velocity_state_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager velocity_state_cb is nullptr...");
        this->velocity_state_cb_flag_ = false;
        return;
    }
}

void ktp::data::RobotStatusManager::gps_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr gps_cb)
{
    this->gps_cb_ = gps_cb;

    if (this->gps_cb_ != nullptr)
    {
        this->gps_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager gps_cb is nullptr...");
        this->gps_cb_flag_ = false;
        return;
    }
}

void ktp::data::RobotStatusManager::rtt_odom_subscription_cb(const geometry_msgs::msg::PoseStamped::SharedPtr rtt_odom_cb)
{
    this->rtt_odom_cb_ = rtt_odom_cb;

    if (this->rtt_odom_cb_ != nullptr)
    {
        this->rtt_odom_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager rtt_odom_cb is nullptr...");
        this->rtt_odom_cb_flag_ = false;
        return;
    }
}

void ktp::data::RobotStatusManager::temperature_subscription_cb(const sensor_msgs::msg::Temperature::SharedPtr temperature_cb)
{
    this->temperature_cb_ = temperature_cb;

    if (this->temperature_cb_ != nullptr)
    {
        this->temperature_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager temperature_cb is nullptr...");
        this->temperature_cb_flag_ = false;
        return;
    }
}

void ktp::data::RobotStatusManager::humidity_subscription_cb(const sensor_msgs::msg::RelativeHumidity::SharedPtr humidity_cb)
{
    this->humidity_cb_ = humidity_cb;

    if (this->humidity_cb_ != nullptr)
    {
        this->humidity_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager humidity_cb is nullptr...");
        this->humidity_cb_flag_ = false;
        return;
    }
}

void ktp::data::RobotStatusManager::robot_navigation_status_from_task_ctrl_cb(const ktp_data_msgs::msg::Status::SharedPtr robot_navigation_cb)
{
    this->robot_navigation_status_cb_ = robot_navigation_cb;

    if (this->robot_navigation_status_cb_ != nullptr)
    {
        this->robot_navigation_status_cb_flag_ = true;
    }
    else
    {
        RCLCPP_ERROR(this->node_->get_logger(), "RobotStatusManager robot_navigation_status_cb is nullptr...");
        this->robot_navigation_status_cb_flag_ = false;
        return;
    }
}

ktp_data_msgs::msg::StatusService ktp::data::RobotStatusManager::build_service()
{
    ktp_data_msgs::msg::StatusService::UniquePtr rbt_status_service = std::make_unique<ktp_data_msgs::msg::StatusService>();

    ktp_data_msgs::msg::StatusServiceEnv::UniquePtr rbt_status_service_env = std::make_unique<ktp_data_msgs::msg::StatusServiceEnv>();

    // ############################################
    // temperature
    // 온도 (단위 : 섭씨)
    // Optional
    // /sensor/temp/temperature
    // RCLCPP_INFO(this->node_->get_logger(), "RobotStatusManager temperature temperature_cb_flag : [%d]", this->temperature_cb_flag_);
    if (this->temperature_cb_flag_)
    {
        rbt_status_service_env->set__temperature(this->temperature_cb_->temperature);
    }
    else
    {
        rbt_status_service_env->set__temperature(DEFAULT_DOUBLE);
    }

    // ############################################
    // humidity
    // 습도 (단위 : percent)
    // Optional
    // /sensor/temp/humidity
    // ############################################
    // RCLCPP_INFO(this->node_->get_logger(), "RobotStatusManager humidity humidity_cb_flag : [%d]", this->humidity_cb_flag_);
    if (this->humidity_cb_flag_)
    {
        rbt_status_service_env->set__humidity(this->humidity_cb_->relative_humidity);
    }
    else
    {
        rbt_status_service_env->set__humidity(DEFAULT_DOUBLE);
    }

    const ktp_data_msgs::msg::StatusServiceEnv &&rbt_status_service_env_moved = std::move(*rbt_status_service_env);

    rbt_status_service->set__env(rbt_status_service_env_moved);
    const ktp_data_msgs::msg::StatusService &&rbt_status_service_moved = std::move(*rbt_status_service);

    return rbt_status_service_moved;
}

ktp_data_msgs::msg::Status ktp::data::RobotStatusManager::build_robot_status()
{
    ktp_data_msgs::msg::Status::UniquePtr rbt_status = std::make_unique<ktp_data_msgs::msg::Status>();

    // ############################################
    // map_id
    // 현재 맵 id
    // 필수
    // 고정 값
    if (this->robot_navigation_status_cb_flag_)
    {
        const std::string &map_id = this->robot_navigation_status_cb_->map_id;
        rbt_status->set__map_id(map_id);
    } else
    {
        rbt_status->set__map_id("");
    }
    // ############################################

    // ############################################
    // battery
    // 배터리 잔여율(%)
    // 필수
    // /sensor/battery/state
    // RCLCPP_INFO(this->node_->get_logger(), "RobotStatusManager percentage battery_cb_flag : [%d]", this->battery_cb_flag_);
    if (this->battery_cb_flag_)
    {
        rbt_status->set__battery(this->battery_state_cb_->voltage);
    } else
    {
        rbt_status->set__battery(DEFAULT_DOUBLE);
    }
    // ############################################

    // ############################################
    // drive_status
    // 로봇의 주행 상태
    // 필수
    // /route_to_pose subscription
    if (this->robot_navigation_status_cb_flag_)
    {
        const int32_t &drive_status = this->robot_navigation_status_cb_->drive_status;
        rbt_status->set__drive_status(drive_status);
    } else
    {
        rbt_status->set__drive_status(ktp::enums::DriveStatus::WAIT);
    }
    // ############################################

    // ############################################
    // create_time
    // 생성 일시
    // 필수
    const std::string &create_time = get_current_time();
    rbt_status->set__create_time(create_time);
    // ############################################

    // ############################################
    // speed
    // 로봇의 주행 속도
    // 필수
    // /sensor/velocity/state
    if (this->velocity_state_cb_flag_)
    {
        const double &speed = this->velocity_state_cb_->current_velocity;
        rbt_status->set__speed(speed);
    } else
    {
        rbt_status->set__speed(DEFAULT_DOUBLE);
    }
    // ############################################

    // ############################################
    // from_node
    // 현재 출발 노드
    // 필수
    // /path_graph/path
    if (this->robot_navigation_status_cb_flag_)
    {
        const std::string &from_node = this->robot_navigation_status_cb_->from_node;
        rbt_status->set__from_node(from_node);
    } else
    {
        rbt_status->set__from_node("");
    }
    // ############################################

    // ############################################
    // floor
    // 로봇이 위치한 층
    // Optional
    rbt_status->set__floor("");
    // ############################################

    // ############################################
    // section
    // Optional
    // 로봇이 위치한 구역
    rbt_status->set__section("");
    // ############################################

    // ############################################
    // to_node
    // 현재 목적 노드
    // 필수
    // /path_graph/path
    if (this->robot_navigation_status_cb_flag_)
    {
        const std::string &to_node = this->robot_navigation_status_cb_->to_node;
        rbt_status->set__to_node(to_node);
    } else
    {
        rbt_status->set__to_node("");
    }
    // ############################################

    // ############################################
    // charge
    // 로봇의 충전 상태
    // 필수
    // /sensor/battery/state
    if (this->battery_cb_flag_)
    {
        const bool &is_charging = this->battery_state_cb_->present;
        rbt_status->set__charge(!is_charging);
    }
    else
    {
        rbt_status->set__charge(false);
    }

    // ############################################

    // ############################################
    // x, y
    // x(longitude), y(latitude) 좌표
    // 필수
    // /sensor/ublox/fix
    // RCLCPP_INFO(this->node_->get_logger(), "RobotStatusManager gps_cb_flag : [%d]", this->gps_cb_flag_);
    if (this->gps_cb_flag_)
    {
        const double &longitude = this->gps_cb_->longitude;
        const double &latitude = this->gps_cb_->latitude;
        rbt_status->set__x(longitude);
        rbt_status->set__y(latitude);
    }
    else
    {
        rbt_status->set__x(DEFAULT_DOUBLE);
        rbt_status->set__y(DEFAULT_DOUBLE);
    };
    // ############################################

    // ############################################
    // heading
    // 로봇의 방향 값(degree 기준)
    // 필수
    // /drive/rtt_odom
    // RCLCPP_INFO(this->node_->get_logger(), "RobotStatusManager rtt_odom_cb_flag : [%d]", this->rtt_odom_cb_flag_);
    if (this->rtt_odom_cb_flag_)
    {
        const double &heading = this->rtt_odom_cb_->pose.orientation.y;
        rbt_status->set__heading(heading);
    }
    else
    {
        rbt_status->set__heading(DEFAULT_DOUBLE);
    }
    // ############################################

    // ############################################
    // is_indoor
    // 실내/실외 구분값
    // 필수
    // ??
    rbt_status->set__is_indoor(false);
    // ############################################

    // ############################################
    // coord_code
    // 로봇 주행 좌표계 코드
    rbt_status->set__coord_code(COORD_CORD_GPS);
    // ############################################

    // ############################################
    // service_mode
    // 서비스 모드
    if (this->robot_navigation_status_cb_flag_)
    {
        const std::string &service_mode = this->robot_navigation_status_cb_->service_mode;
        rbt_status->set__service_mode(service_mode);
    }
    else
    {
        rbt_status->set__service_mode("");
    }
    // ############################################

    // ############################################
    // service
    // 서비스 특화 데이터
    const ktp_data_msgs::msg::StatusService &rbt_status_service = this->build_service();
    rbt_status->set__service(rbt_status_service);
    // ############################################

    // ############################################
    // firmware_version
    rbt_status->set__firmware_version("1.0.5.21");
    // ############################################

    // RCLCPP_INFO(
    //     this->node_->get_logger(),
    //     "RobotStatusManager\n\tbattery : [%f]\n\tcreate_time : [%s]\n\tlongitude : [%f]\n\tlatitude : [%f]\n\theading : [%f]",
    //     rbt_status->battery,
    //     CSTR(create_time),
    //     longitude,
    //     latitude,
    //     heading);

    const ktp_data_msgs::msg::Status &&rbt_status_moved = std::move(*rbt_status);

    return rbt_status_moved;
}