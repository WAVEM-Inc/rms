#ifndef DETECTED_OBJECT_MANAGER__HXX
#define DETECTED_OBJECT_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class DetectedObjectManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;



        public:
            explicit DetectedObjectManager(rclcpp::Node::SharedPtr node);
            virtual ~DetectedObjectManager();

        public:
            using SharedPtr = std::shared_ptr<DetectedObjectManager>;
        };
    }

}

#endif // DETECTED_OBJECT_MANAGER__HXX