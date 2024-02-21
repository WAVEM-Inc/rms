#ifndef CONTROL_NOTIFICATOR__HXX
#define CONTROL_NOTIFICATOR__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include "controller/assignment/assignment_controller.hxx"
#include "controller/assignment/control/control_assigner.hxx"

#define DEFAULT_QOS 10

#define NOTIFICATE_MISSION_TO_MGR_TOPIC "/ktp/task/notificate/mission"
#define NOTIFICATE_CONTROL_TO_MGR_TOPIC "/ktp/task/notificate/control"

#define NOTIFICATE_PUBLISHING_RATE 850

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace controller
    {
        class ControlNotificator final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::controller::AssignmentController::SharedPtr assignment_controller_;

            rclcpp::CallbackGroup::SharedPtr notificate_control_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Control>::SharedPtr notificate_control_publihser_;

        public:
            explicit ControlNotificator(rclcpp::Node::SharedPtr node);
            virtual ~ControlNotificator();
            void notificate_control();

        public:
            using SharedPtr = std::shared_ptr<ControlNotificator>;
        };
    }
}

#endif // CONTROL_NOTIFICATOR__HXX