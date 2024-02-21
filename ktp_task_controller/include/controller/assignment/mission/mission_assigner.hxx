#ifndef MISSION_ASSIGNER__HXX
#define MISSION_ASSIGNER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/srv/assign_mission.hpp>

#include "controller/assignment/mission/mission_assigner.hxx"

#include "domain/mission/mission.hxx"

#define DEFAULT_QOS 10

#define MISSION_ASSIGN_FROM_ITF_TOPIC "/ktp/task/mission/assign"

#define ASSIGN_MISSION_SERVICE_NAME "/ktp_task_controller/assign/mission"

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

            rclcpp::Service<ktp_data_msgs::srv::AssignMission>::SharedPtr assign_mission_service_;
            void assign_mission_service_cb(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Request> request,
                const std::shared_ptr<ktp_data_msgs::srv::AssignMission::Response> response);

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