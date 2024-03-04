#ifndef CONTROL_REPORT_MANAGER__HXX
#define CONTROL_REPORT_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/control_report.hpp>
#include <ktp_data_msgs/msg/control_report_data.hpp>
#include <ktp_data_msgs/msg/control_report_data_graph_list.hpp>

#define DEFAULT_QOS 10

#define CONTROL_REPORT_TO_ITF_TOPIC "/rms/ktp/data/control_report"
#define NOTIFY_MISSION_FROM_TASK_CTRL_TOPIC "/rms/ktp/task/notify/mission"
#define NOTIFY_CONTROL_FROM_TASK_CTRL_TOPIC "/rms/ktp/task/notify/control"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class ControlReportManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr control_report_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ControlReport>::SharedPtr control_report_to_itf_publisher_;

            rclcpp::CallbackGroup::SharedPtr notify_mission_from_task_ctrl_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::ControlReport>::SharedPtr notify_mission_from_task_ctrl_subscription_;
            void notify_mission_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ControlReport::SharedPtr notify_mission_cb);

        public:
            explicit ControlReportManager(rclcpp::Node::SharedPtr node);
            virtual ~ControlReportManager();

        public:
            using SharedPtr = std::shared_ptr<ControlReportManager>;
        };
    }
}

#endif // CONTROL_REPORT_MANAGER__HXX