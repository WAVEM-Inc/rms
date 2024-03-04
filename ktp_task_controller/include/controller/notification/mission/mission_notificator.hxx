#ifndef MISSION_NOTIFICATOR__HXX
#define MISSION_NOTIFICATOR__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/control_report.hpp>

#include "domain/mission/mission.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10

#define NOTIFY_MISSION_TO_MGR_TOPIC "/rms/ktp/task/notify/mission"
#define NOTIFY_CONTROL_TO_MGR_TOPIC "/rms/ktp/task/notify/control"

#define CONTROL_TYPE_MISSION "mission"

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

            rclcpp::CallbackGroup::SharedPtr notify_mission_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ControlReport>::SharedPtr notify_mission_publisher_;

        public:
            explicit MissionNotificator(rclcpp::Node::SharedPtr node);
            virtual ~MissionNotificator();
            void notify_mission_status(ktp::domain::Mission::SharedPtr domain_mission);

        public:
            using SharedPtr = std::shared_ptr<MissionNotificator>;
        };
    }
}

#endif // MISSION_NOTIFICATOR__HXX