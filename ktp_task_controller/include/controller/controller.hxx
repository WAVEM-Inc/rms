#ifndef CONTROLLER__HXX
#define CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rcutils/logging_macros.h>

#include "controller/assignment/assignment_controller.hxx"
#include "controller/notification/notification_controller.hxx"

#define NODE_NAME "ktp_task_controller"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace controller
    {
        class MainController final : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::controller::AssignmentController::SharedPtr assignment_controller_;
            ktp::controller::NotificationController::SharedPtr notification_controller_;
        public:
            explicit MainController();
            virtual ~MainController();
        };
    }
}

#endif