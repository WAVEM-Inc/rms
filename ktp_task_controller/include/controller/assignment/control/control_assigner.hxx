#ifndef CONTROL_ASSIGNER__HXX
#define CONTROL_ASSIGNER__HXX

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/control.hpp>
#include <ktp_data_msgs/srv/assign_control.hpp>

#include "domain/control/control.hxx"

#define ASSIGN_CONTROL_SERVICE_NAME "/ktp_task_controller/assign/control"

#define CSTR(str) ((str).c_str())

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ktp
{
    namespace controller
    {
        class ControlAssigner final
        {
        private:
            rclcpp::Node::SharedPtr node_;
            ktp::domain::Control::SharedPtr control_;

            rclcpp::Service<ktp_data_msgs::srv::AssignControl>::SharedPtr assign_control_service_;
            void assign_control_service_cb(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Request> request,
                const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Response> response);

        public:
            explicit ControlAssigner(rclcpp::Node::SharedPtr node);
            virtual ~ControlAssigner();
            ktp::domain::Control::SharedPtr transmiss_control_to_notification();

        public:
            using SharedPtr = std::shared_ptr<ControlAssigner>;
        };
    }
}

#endif // CONTROL_ASSIGNER__HXX