#ifndef MISSION_NOTIFICATOR__HXX
#define MISSION_NOTIFICATOR__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/status.hpp>

#include <ktp_data_msgs/msg/service_status.hpp>
#include <ktp_data_msgs/msg/service_status_task.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/control_report.hpp>

#include <route_msgs/msg/path.hpp>

#include "domain/mission/mission.hxx"
#include "utils/utils.hxx"

#define DEFAULT_QOS 10

#define DEFAULT_INT 0

#define MISSION_ASSIGN_FAILED_CODE 5000

#define ROUTE_TO_POSE_FEEDBACK_NAVIGATION_STARTED_CODE 1001
#define ROUTE_TO_POSE_FEEDBACK_ON_PROGRESS_CODE 2001
#define ROUTE_TO_POSE_FEEDBACK_LIDAR_OBJECT_DETECTED_CODE 3001
#define ROUTE_TO_POSE_FEEDBACK_COOPERATIVE_OBJECT_DETECTED_CODE 3002
#define ROUTE_TO_POSE_FEEDBACK_NAVIGATION_SUCCEEDED_CODE 4001
#define ROUTE_TO_POSE_FEEDBACK_NAVIGATION_CANCELED_CODE 5001

#define ROUTE_TO_POSE_RESULT_SUCCEEDED_CODE 1001
#define ROUTE_TO_POSE_RESULT_NAVIGATION_CANCELED_CODE 2001
#define ROUTE_TO_POSE_RESULT_NAVIGATION_UNKNOWN_ROUTE_CODE 2002
#define ROUTE_TO_POSE_RESULT_NAVIGATION_LONG_TERM_WAITING_CODE 2003
#define ROUTE_TO_POSE_RESULT_SYSTEM_DISABLED_CODE 2004
#define ROUTE_TO_POSE_RESULT_ABORTED_CODE 3001

#define MISSION_TASK_STARTED_STATUS "Started"
#define MISSION_TASK_STARTED_CODE 0

#define MISSION_TASK_SOURCE_ARRIVED_STATUS "SourceArrived"
#define MISSION_TASK_SOURCE_ARRIVED_CODE 1

#define MISSION_TASK_TAKEN_STATUS "Taken"
#define MISSION_TASK_TAKEN_CODE 2

#define MISSION_TASK_ON_PROGRESS_STATUS "OnProgress"
#define MISSION_TASK_ON_PROGRESS_CODE 3

#define MISSION_TASK_DEST_ARRIVED_STATUS "DestArrived"
#define MISSION_TASK_DEST_ARRIVED_CODE 4

#define MISSION_TASK_END_STATUS "End"
#define MISSION_TASK_ENDED_CODE 5

#define MISSION_TASK_CANCELLED_STATUS "Cancelled"
#define MISSION_TASK_CANCELLED_CODE 6

#define MISSION_TASK_FAILED_STATUS "Failed"
#define MISSION_TASK_FAILED_CODE 7

#define NOTIFY_MISSION_REPORT_TO_MGR_TOPIC "/rms/ktp/task/notify/mission/report"
#define NOTIFY_MISSION_STATUS_TO_MGR_TOPIC "/rms/ktp/task/notify/mission/status"
#define NOTIFY_ROBOT_NAVIGATION_STATUS_TO_MGR_TOPIC "/rms/ktp/task/notify/robot_navigation/status"

#define CONTROL_TYPE_MISSION "mission"

#define DEVICE_ID "KECDSEMITB001"
#define MISSION_CODE "ktdelivery"
#define MISSION_OWNER "padmin"

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

            rclcpp::CallbackGroup::SharedPtr notify_mission_status_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ServiceStatus>::SharedPtr notify_mission_status_publisher_;

            rclcpp::CallbackGroup::SharedPtr notify_robot_navigation_status_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::Status>::SharedPtr notify_robot_navigation_status_publisher_;

        public:
            explicit MissionNotificator(rclcpp::Node::SharedPtr node);
            virtual ~MissionNotificator();
            void notify_mission_status(int32_t status_code, ktp_data_msgs::msg::MissionTask mission_task);
            void notify_robot_navigation_status(ktp::domain::NavigationStatus navigation_status);

        public:
            using SharedPtr = std::shared_ptr<MissionNotificator>;
        };
    }
}

#endif // MISSION_NOTIFICATOR__HXX