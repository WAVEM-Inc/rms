#ifndef MANAGER__HXX
#define MANAGER__HXX

#include "manager/request/request_manager.hxx"
#include "manager/response/response_manager.hxx"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rcutils/logging_macros.h>

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

#define NODE_NAME "ktp_data_manager"
#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class Manager : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::data::RequestManager::SharedPtr request_manager_;
            ktp::data::ResponseManager::SharedPtr response_manager_;

        public:
            explicit Manager();
            virtual ~Manager();

        public:
            using SharedPtr = std::shared_ptr<Manager>;
        };
    }
}

#endif