#ifndef MISSION_ASSIGNER__HXX
#define MISSION_ASSIGNER__HXX

#include <vector>
#include <iterator>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>
#include <ktp_data_msgs/srv/assign_mission.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <path_graph_msgs/srv/path.hpp>
#include <route_msgs/msg/path.hpp>
#include <route_msgs/msg/node.hpp>
#include <route_msgs/action/route_to_pose.hpp>

#include "controller/notification/mission/mission_notificator.hxx"

#include "domain/mission/mission.hxx"

#include "exception/assignment/assignment_exceptions.hxx"

#define DEFAULT_INT 0
#define DEFAULT_DOUBLE 0.0
#define DEFAULT_QOS 10

#define MISSION_RECEPTION_SUCCEEDED_CODE 200
#define MISSION_RECEPTION_FAILED_CODE 400

#define NODE_WORKPLACE_NODE_TYPE "workplace_node"

#define MISSION_ASSIGN_FAILED_CODE 5000

#define ROUTE_TO_POSE_FEEDBACK_NAVIGATION_STARTED_CODE 1001
#define ROUTE_TO_POSE_FEEDBACK_ON_PROGRESS_CODE 2001
#define ROUTE_TO_POSE_FEEDBACK_LIDAR_OBJECT_DETECTED_CODE 3001
#define ROUTE_TO_POSE_FEEDBACK_COOPERATIVE_OBJECT_DETECTED_CODE 3002
#define ROUTE_TO_POSE_FEEDBACK_LIDAR_DETECTED_WAIT_CODE 4001
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

#define ASSIGN_MISSION_SERVICE_NAME "/ktp_task_controller/assign/mission"
#define UBLOX_FIX_TOPIC "/sensor/ublox/fix"
#define PATH_GRAPH_PATH_SERVICE_NAME "/path_graph/path"
#define ROUTE_TO_POSE_ACTION_NAME "/route_to_pose"

#define CSTR(str) ((str).c_str())

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ktp
{
    namespace controller
    {
        class MissionAssigner final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::domain::Mission::SharedPtr domain_mission_;

            std::vector<ktp_data_msgs::msg::MissionTask> task_vec_;
            int task_current_index_;
            int task_vec_size_;

            std::vector<route_msgs::msg::Path> path_vec_;
            int path_current_index_;
            int path_vec_size_;

            int node_current_index_;
            int node_list_size_;

            ktp::controller::MissionNotificator::SharedPtr mission_notificator_;

            // 0
            bool is_waiting_area_to_top_chart_proceeding = false;

            // 1
            bool is_top_chart_to_drop_off_proceeding = false;

            // 2
            bool is_drop_off_to_waiting_area_proceeding = false;

            bool ublox_fix_cb_flag_ = false;
            sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_cb_;
            rclcpp::CallbackGroup::SharedPtr ublox_fix_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ublox_fix_subscription_;
            void ublox_fix_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_cb);

            rclcpp::CallbackGroup::SharedPtr assign_mission_service_cb_group_;
            rclcpp::Service<ktp_data_msgs::srv::AssignMission>::SharedPtr assign_mission_service_;
            void assign_mission_service_cb(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response);

            rclcpp::CallbackGroup::SharedPtr path_graph_path_service_client_cb_group_;
            rclcpp::Client<path_graph_msgs::srv::Path>::SharedPtr path_graph_path_service_client_;
            bool request_converting_goal_to_path(path_graph_msgs::srv::Path::Request::SharedPtr path_request);

            path_graph_msgs::srv::Path::Request::SharedPtr build_waiting_area_to_top_chart_path_request();
            path_graph_msgs::srv::Path::Request::SharedPtr build_top_chart_to_drop_off_path_request();
            path_graph_msgs::srv::Path::Request::SharedPtr build_drop_off_to_waiting_area_path_request();

            rclcpp::CallbackGroup::SharedPtr route_to_pose_action_client_cb_group_;
            rclcpp_action::Client<route_msgs::action::RouteToPose>::SharedPtr route_to_pose_action_client_;
            void route_to_pose_goal_response_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr &goal_handle);
            void route_to_pose_feedback_cb(rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr goal_handle, const std::shared_ptr<const route_msgs::action::RouteToPose::Feedback> feedback);
            void route_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::WrappedResult &result);

        public:
            explicit MissionAssigner(rclcpp::Node::SharedPtr node);
            virtual ~MissionAssigner();
            void route_to_pose_send_goal();

        public:
            using SharedPtr = std::shared_ptr<MissionAssigner>;
        };
    }
}

#endif // MISSION_ASSIGNER__HXX