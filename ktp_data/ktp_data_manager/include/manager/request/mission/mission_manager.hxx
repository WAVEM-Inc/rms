#ifndef MISSION_MANAGER__HXX
#define MISSION_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/srv/assign_mission.hpp>

#define DEFAULT_QOS 10

#define MISSION_FROM_ITF_TOPIC "/rms/ktp/data/mission"
#define MISSION_TO_TASK_CTRL_SERVICE_NAME "/ktp_task_controller/assign/mission"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class MissionManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr mission_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Mission>::SharedPtr mission_from_itf_subscription_;
            void mission_from_itf_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb);

            rclcpp::CallbackGroup::SharedPtr assign_mission_client_cb_group_;
            rclcpp::Client<ktp_data_msgs::srv::AssignMission>::SharedPtr assign_mission_client_;
            void assign_mission_service_req(const ktp_data_msgs::msg::Mission::SharedPtr mission_from_itf);

        public:
            explicit MissionManager(rclcpp::Node::SharedPtr node);
            virtual ~MissionManager();

        public:
            using SharedPtr = std::shared_ptr<MissionManager>;
        };
    }

}

#endif // MISSION_MANAGER__HXX