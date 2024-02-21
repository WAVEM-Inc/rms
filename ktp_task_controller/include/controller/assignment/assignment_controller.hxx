#ifndef ASSIGNMENT_CONTROLLER__HXX
#define ASSIGNMENT_CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/control.hpp>

#include <ktp_data_msgs/srv/assign_mission.hpp>
#include <ktp_data_msgs/srv/assign_control.hpp>

#include "controller/assignment/mission/mission_assigner.hxx"
#include "controller/assignment/control/control_assigner.hxx"

#include "domain/mission/mission.hxx"
#include "domain/control/control.hxx"

#define DEFAULT_QOS 10

#define MISSION_ASSIGN_FROM_ITF_TOPIC "/ktp/task/mission/assign"

#define ASSIGN_MISSION_SERVICE_NAME "/ktp_task_controller/assign/mission"
#define ASSIGN_CONTROL_SERVICE_NAME "/ktp_task_controller/assign/control"

#define CSTR(str) ((str).c_str())

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ktp
{
    namespace controller
    {
        class AssignmentController final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::controller::MissionAssigner::SharedPtr mission_assigner_;
            ktp::controller::ControlAssigner::SharedPtr control_assigner_;

        public:
            explicit AssignmentController(rclcpp::Node::SharedPtr node);
            virtual ~AssignmentController();
            ktp_data_msgs::msg::Mission transmiss_mission_to_notification();
            ktp_data_msgs::msg::Control transmiss_control_to_notification();
            
        public:
            using SharedPtr = std::shared_ptr<AssignmentController>;
        };
    }
}

#endif // ASSIGNMENT_CONTROLLER__HXX