#ifndef BUILDER__HXX
#define BUILDER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/status.hpp>
#include <ktp_data_msgs/msg/status_service.hpp>
#include <ktp_data_msgs/msg/status_service_env.hpp>

#include <ktp_data_msgs/msg/service_status.hpp>
#include <ktp_data_msgs/msg/service_status_task.hpp>
#include <ktp_data_msgs/msg/service_status_task_data.hpp>

#include <ktp_data_msgs/msg/error_report.hpp>

#include <ktp_data_msgs/msg/control_report.hpp>
#include <ktp_data_msgs/msg/control_report_data.hpp>
#include <ktp_data_msgs/msg/control_report_data_graph_list.hpp>

#include <ktp_data_msgs/msg/control.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/graph_list.hpp>
#include <ktp_data_msgs/msg/graph.hpp>
#include <ktp_data_msgs/msg/graph_node_list.hpp>
#include <ktp_data_msgs/msg/graph_edge_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_vertices.hpp>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "model/model_enums.hxx"

#include "builder/control/control_builder.hxx"
#include "builder/control_report/control_report_builder.hxx"
#include "builder/error_report/error_report_builder.hxx"
#include "builder/graph_list/graph_list_builder.hxx"
#include "builder/mission/mission_builder.hxx"
#include "builder/robot_status/robot_status_builder.hxx"
#include "builder/service_status/service_status_builder.hxx"

#define DEFAULT_QOS 10

#define BATTERY_STATE_TOPIC "/battery/state"
#define VELOCITY_STATE_TOPIC "/velocity/state"
#define GPS_TOPIC "/ublox/fix"
#define RTT_ODOM_TOPIC "/rtt_odom"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class MainBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::build::RobotStatusBuilder::SharedPtr robot_status_builder_;
            // ktp::build::ServiceStatusBuilder::SharedPtr service_status_builder_;
            // ktp::build::ErrorReportBuilder::SharedPtr error_report_builder_;

            // std::shared_ptr<ktp::build::ControlBuilder> control_builder_;
        public:
            explicit MainBuilder(rclcpp::Node::SharedPtr node);
            virtual ~MainBuilder();
            ktp_data_msgs::msg::Status build_rbt_status();
            ktp_data_msgs::msg::ServiceStatus build_rbt_service_status();
        public:
            using SharedPtr = std::shared_ptr<MainBuilder>;
        };
    }
}

#endif