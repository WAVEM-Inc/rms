#ifndef RESOURCE_MODEL__HXX
#define RESOURCE_MODEL__HXX

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

namespace ktp
{
    namespace data
    {
        class ResourceModel
        {
        private:
            ktp_data_msgs::msg::Status::SharedPtr rbt_status_;
            ktp_data_msgs::msg::ServiceStatus::SharedPtr rbt_service_status_;
            ktp_data_msgs::msg::ErrorReport::SharedPtr rbt_error_report_;
            ktp_data_msgs::msg::Control::SharedPtr rbt_control_;
            ktp_data_msgs::msg::ControlReport::SharedPtr rbt_control_report_;
            ktp_data_msgs::msg::Mission::SharedPtr rbt_mission_;
            ktp_data_msgs::msg::GraphList::SharedPtr rbt_graph_list_;

        public:
            explicit ResourceModel();
            virtual ~ResourceModel();

            ktp_data_msgs::msg::Status get__rbt_status();
            void set__rbt_status(ktp_data_msgs::msg::Status::SharedPtr rbt_status);

            ktp_data_msgs::msg::ServiceStatus get__rbt_service_status();
            ktp_data_msgs::msg::ErrorReport get__rbt_error_report();
            ktp_data_msgs::msg::Control get__rbt_control();
            ktp_data_msgs::msg::ControlReport get__rbt_control_report();
            ktp_data_msgs::msg::Mission get__rbt_mission();
            ktp_data_msgs::msg::GraphList get__rbt_graph_list();
        };
    }
}

#endif