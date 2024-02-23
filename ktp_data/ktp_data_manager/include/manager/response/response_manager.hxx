#ifndef RESPONSE_MANAGER__HXX
#define RESPONSE_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include "manager/response/robot_status/robot_status_manager.hxx"
#include "manager/response/service_status/service_status_manager.hxx"
#include "manager/response/error_report/error_report_manager.hxx"
#include "manager/response/control_report/control_report_manager.hxx"
#include "manager/response/graph_list/graph_list_manager.hxx"

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class ResponseManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::data::RobotStatusManager::SharedPtr robot_status_manager_;
            ktp::data::ServiceStatusManager::SharedPtr service_status_manager_;
            ktp::data::ErrorReportManager::SharedPtr error_report_manager_;
            ktp::data::ControlReportManager::SharedPtr control_report_manager_;
            ktp::data::GraphListManager::SharedPtr graph_list_manager_;

        public:
            explicit ResponseManager(rclcpp::Node::SharedPtr node);
            virtual ~ResponseManager();

        public:
            using SharedPtr = std::shared_ptr<ResponseManager>;
        };
    }
}

#endif // RESPONSE_MANAGER__HXX