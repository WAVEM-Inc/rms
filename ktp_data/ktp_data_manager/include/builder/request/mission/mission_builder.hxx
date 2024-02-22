#ifndef MISSION_BUILDER__HXX
#define MISSION_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include "model/model_enums.hxx"

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class MissionBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit MissionBuilder(rclcpp::Node::SharedPtr node);
            virtual ~MissionBuilder();
        public:
            using SharedPtr = std::shared_ptr<MissionBuilder>;
        };
    }
}

#endif // MISSION_BUILDER__HXX