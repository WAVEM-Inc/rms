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
#include <path_graph_msgs/msg/path.hpp>
#include <path_graph_msgs/msg/position.hpp>
#include <path_graph_msgs/msg/node.hpp>
#include <path_graph_msgs/srv/path.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
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

#define MISSION_ASSIGN_FROM_ITF_TOPIC "/ktp/task/mission/assign"
#define ASSIGN_MISSION_SERVICE_NAME "/ktp_task_controller/assign/mission"
// #define UBLOX_FIX_TOPIC "/sensor/ublox/fix"
#define UBLOX_FIX_TOPIC "/sensor/ublox/fix"
#define PATH_GRAPH_PATH_SERVICE_NAME "/path_graph/path"
// #define ROUTE_TO_POSE_ACTION_NAME "/navigate_to_pose"
// #define ROUTE_TO_POSE_STATUS_TOPIC_NAME "/navigate_to_pose/_action/status"
#define ROUTE_TO_POSE_ACTION_NAME "/route_to_pose"
#define ROUTE_TO_POSE_FEEDBACK_TOPIC_NAME "/route_to_pose/_action/feedback"
#define ROUTE_TO_POSE_STATUS_TOPIC_NAME "/route_to_pose/_action/status"

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

            ktp::domain::Mission::SharedPtr mission_;

            std::vector<ktp_data_msgs::msg::MissionTask> task_vec_;
            int task_current_index_;
            int task_vec_size_;

            std::vector<route_msgs::msg::Path> path_vec_;
            int path_current_index_;
            int path_vec_size_;

            int node_current_index_;
            int node_list_size_;

            ktp::controller::MissionNotificator::SharedPtr mission_notificator_;

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
            bool request_converting_goal_to_path();

            rclcpp::CallbackGroup::SharedPtr route_to_pose_action_client_cb_group_;
            rclcpp_action::Client<route_msgs::action::RouteToPose>::SharedPtr route_to_pose_action_client_;
            void route_to_pose_send_goal();
            void route_to_pose_goal_response_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr &goal_handle);
            void route_to_pose_feedback_cb(rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::SharedPtr goal_handle, const std::shared_ptr<const route_msgs::action::RouteToPose::Feedback> feedback);
            void route_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<route_msgs::action::RouteToPose>::WrappedResult &result);

            rclcpp::CallbackGroup::SharedPtr route_to_pose_status_subscription_cb_group_;
            rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr route_to_pose_status_subscription_;
//            void route_to_pose_status_subscription_cb(const action_msgs::msg::GoalStatusArray::SharedPtr route_to_pose_status_cb);

        public:
            explicit MissionAssigner(rclcpp::Node::SharedPtr node);
            virtual ~MissionAssigner();

        public:
            using SharedPtr = std::shared_ptr<MissionAssigner>;
        };
    }
}

#endif // MISSION_ASSIGNER__HXX