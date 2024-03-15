#ifndef CONTROL_MANAGER__HXX
#define CONTROL_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/control.hpp>

#include <ktp_data_msgs/srv/assign_control.hpp>

#define DEFAULT_QOS 10

#define CONTROL_FROM_ITF_TOPIC "/rms/ktp/data/control"

#define ASSIGN_CONTROL_FROM_ITF_SERVICE_NAME "/ktp_data_manager/assign/control"
#define ASSIGN_CONTROL_TO_TASK_CTRL_SERVICE_NAME "/ktp_task_controller/assign/control"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ktp
{
    namespace data
    {
        class ControlManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr assign_control_from_itf_service_cb_group_;
            rclcpp::Service<ktp_data_msgs::srv::AssignControl>::SharedPtr assign_control_from_itf_service_;
            void assign_control_from_itf_service_cb(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Request> request,
                const std::shared_ptr<ktp_data_msgs::srv::AssignControl::Response> response
                );

            rclcpp::CallbackGroup::SharedPtr assign_control_to_task_ctrl_service_client_cb_group_;
            rclcpp::Client<ktp_data_msgs::srv::AssignControl>::SharedPtr assign_control_to_task_ctrl_service_client_;
            bool request_assign_control_to_task_ctrl(ktp_data_msgs::srv::AssignControl::Request::SharedPtr request);

        public:
            explicit ControlManager(rclcpp::Node::SharedPtr node);
            virtual ~ControlManager();
        public:
            using SharedPtr = std::shared_ptr<ControlManager>;
        };
    }

}

#endif // CONTROL_MANAGER__HXX