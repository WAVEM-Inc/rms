#ifndef CONTROL_NOTIFICATOR__HXX
#define CONTROL_NOTIFICATOR__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/control.hpp>
#include <ktp_data_msgs/msg/control_report.hpp>

#include "utils/utils.hxx"

#define DEFAULT_QOS 10

#define NOTIFY_CONTROL_REPORT_TO_MGR_TOPIC "/rms/ktp/task/notify/control/report"

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

            rclcpp::CallbackGroup::SharedPtr notify_control_report_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ControlReport>::SharedPtr notify_control_report_publisher_;

        public:
            explicit ControlNotificator(rclcpp::Node::SharedPtr node);
            virtual ~ControlNotificator();
            void notify_control_report(ktp_data_msgs::msg::Control control, int response_code);

        public:
            using SharedPtr = std::shared_ptr<ControlNotificator>;
        };
    }
}

#endif // CONTROL_NOTIFICATOR__HXX