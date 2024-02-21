#ifndef NOTIFICATION_CONTROLLER__HXX
#define NOTIFICATION_CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include "controller/assignment/assignment_controller.hxx"

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
        class NotificationController final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::controller::AssignmentController::SharedPtr assignment_controller_;
            
            rclcpp::CallbackGroup::SharedPtr notificate_timer_cb_group_;
            rclcpp::TimerBase::SharedPtr notificate_timer_;
            void notificate_timer_cb();

            rclcpp::CallbackGroup::SharedPtr notificate_mission_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Mission>::SharedPtr notificate_mission_publihser_;

            rclcpp::CallbackGroup::SharedPtr notificate_control_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Control>::SharedPtr notificate_control_publihser_;
        public:
            explicit NotificationController(rclcpp::Node::SharedPtr node);
            virtual ~NotificationController();
        public:
            using SharedPtr = std::shared_ptr<NotificationController>;
        };
    }
}

#endif // NOTIFICATION_CONTROLLER__HXX