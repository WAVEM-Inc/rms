#ifndef MISSION_NOTIFICATOR__HXX
#define MISSION_NOTIFICATOR__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include "controller/assignment/assignment_controller.hxx"

#define DEFAULT_QOS 10

#define NOTIFICATE_MISSION_TO_MGR_TOPIC "/rms/ktp/task/notificate/mission"
#define NOTIFICATE_CONTROL_TO_MGR_TOPIC "/rms/ktp/task/notificate/control"

#define NOTIFICATE_PUBLISHING_RATE 850

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace controller
    {
        class MissionNotificator final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::controller::AssignmentController::SharedPtr assignment_controller_;

            rclcpp::CallbackGroup::SharedPtr notificate_mission_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Mission>::SharedPtr notificate_mission_publihser_;

        public:
            explicit MissionNotificator(rclcpp::Node::SharedPtr node);
            virtual ~MissionNotificator();
            void notificate_mission();

        public:
            using SharedPtr = std::shared_ptr<MissionNotificator>;
        };
    }
}

#endif // MISSION_NOTIFICATOR__HXX