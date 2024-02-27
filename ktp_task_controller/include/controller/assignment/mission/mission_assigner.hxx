#ifndef MISSION_ASSIGNER__HXX
#define MISSION_ASSIGNER__HXX

#include <exception>
#include <stdexcept>

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
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "controller/assignment/mission/mission_assigner.hxx"

#include "domain/mission/mission.hxx"

#define DEFAULT_INT 0
#define DEFAULT_DOUBLE 0.0
#define DEFAULT_QOS 10

#define MISSION_ASSIGN_FROM_ITF_TOPIC "/ktp/task/mission/assign"
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
            ktp::domain::Mission::SharedPtr mission_;
            std::vector<ktp_data_msgs::msg::MissionTaskData> mission_task_data_vec_;
            int mission_task_data_current_idx_;
            int mission_task_data_last_idx_;

            rclcpp::CallbackGroup::SharedPtr assign_mission_service_cb_group_;
            rclcpp::Service<ktp_data_msgs::srv::AssignMission>::SharedPtr assign_mission_service_;
            void assign_mission_service_cb(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response);
            
            bool ublox_fix_cb_flag_ = false;
            sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_cb_;
            rclcpp::CallbackGroup::SharedPtr ublox_fix_subscription_cb_group_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr ublox_fix_subscription_;
            void ublox_fix_subscription_cb(const sensor_msgs::msg::NavSatFix::SharedPtr ublox_fix_cb);

            rclcpp::CallbackGroup::SharedPtr path_graph_path_service_client_cb_group_;
            rclcpp::Client<path_graph_msgs::srv::Path>::SharedPtr path_graph_path_service_client_;
            path_graph_msgs::msg::Path path_graph_path_service_req(ktp_data_msgs::msg::MissionTaskData mission_task_data);

            rclcpp::CallbackGroup::SharedPtr route_to_pose_action_client_cb_group_;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr route_to_pose_action_client_;
            void route_to_pose_send_goal();
            void route_to_pose_goal_response_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle);
            void route_to_pose_feedback_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
            void route_to_pose_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

        public:
            explicit MissionAssigner(rclcpp::Node::SharedPtr node);
            virtual ~MissionAssigner();
            ktp::domain::Mission::SharedPtr transmiss_mission_to_notification();

        public:
            using SharedPtr = std::shared_ptr<MissionAssigner>;
        };
    }
}

#endif // MISSION_ASSIGNER__HXX