#ifndef MISSION_MANAGER__HXX
#define MISSION_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/srv/assign_mission.hpp>

#include <std_msgs/msg/bool.hpp>

#define DEFAULT_QOS 10

#define ASSIGN_MISSION_FROM_ITF_SERVICE_NAME "/ktp_data_manager/assign/mission"
#define ASSIGN_MISSION_TO_TASK_CTRL_SERVICE_NAME "/ktp_task_controller/assign/mission"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ktp
{
    namespace data
    {
        class MissionManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;
            bool mission_in_progress_flag_ = false;

            rclcpp::CallbackGroup::SharedPtr assign_mission_from_itf_service_cb_group_;
            rclcpp::Service<ktp_data_msgs::srv::AssignMission>::SharedPtr assign_mission_from_itf_service_;
            void assign_mission_from_itf_service_cb(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response);

            rclcpp::CallbackGroup::SharedPtr assign_mission_client_cb_group_;
            rclcpp::Client<ktp_data_msgs::srv::AssignMission>::SharedPtr assign_mission_client_;
            bool assign_mission_service_req(const ktp_data_msgs::msg::Mission mission_request_from_itf);

        public:
            explicit MissionManager(rclcpp::Node::SharedPtr node);
            virtual ~MissionManager();

        public:
            using SharedPtr = std::shared_ptr<MissionManager>;
        };
    }

}

#endif // MISSION_MANAGER__HXX