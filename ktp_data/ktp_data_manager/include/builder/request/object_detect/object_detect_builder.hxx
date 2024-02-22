#ifndef OBJECT_DETECT_BUILDER__HXX
#define OBJECT_DETECT_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/object_detect.hpp>

namespace ktp
{
    namespace build
    {
        class ObjectDetectBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit ObjectDetectBuilder(rclcpp::Node::SharedPtr node);
            virtual ~ObjectDetectBuilder();
            ktp_data_msgs::msg::ObjectDetect build_object_detect();
        public:
            using SharedPtr = std::shared_ptr<ObjectDetectBuilder>;
        };
    }
}

#endif // OBJECT_DETECT_BUILDER__HXX