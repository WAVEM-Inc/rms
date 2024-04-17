#ifndef REQUEST_MANAGER__HXX
#define REQUEST_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include "manager/request/control/control_manager.hxx"
#include "manager/request/mission/mission_manager.hxx"
#include "manager/request/detected_object/detected_object_manager.hxx"


#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class RequestManager
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::data::ControlManager::SharedPtr control_manager_;
            ktp::data::MissionManager::SharedPtr mission_manager_;
            ktp::data::DetectedObjectManager::SharedPtr detected_object_manager_;

        public:
            explicit RequestManager(rclcpp::Node::SharedPtr node);
            virtual ~RequestManager();

        public:
            using SharedPtr = std::shared_ptr<RequestManager>;
        };
    }
}

#endif