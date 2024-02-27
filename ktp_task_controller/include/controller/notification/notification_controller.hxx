#ifndef NOTIFICATION_CONTROLLER__HXX
#define NOTIFICATION_CONTROLLER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include "controller/notification/mission/mission_notificator.hxx"
#include "controller/notification/control/control_notificator.hxx"


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
        class NotificationController final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::controller::MissionNotificator::SharedPtr mission_notificator_;
            ktp::controller::ControlNotificator::SharedPtr control_notificator_;
            
        public:
            explicit NotificationController(rclcpp::Node::SharedPtr node);
            virtual ~NotificationController();
        public:
            using SharedPtr = std::shared_ptr<NotificationController>;
        };
    }
}

#endif // NOTIFICATION_CONTROLLER__HXX