#ifndef REQUEST_BUILDER__HXX
#define REQUEST_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>

#include "builder/request/control/control_builder.hxx"
#include "builder/request/mission/mission_builder.hxx"
#include "builder/request/object_detect/object_detect_builder.hxx"

namespace ktp
{
    namespace build
    {
        class RequestBuilder final
        {
        private:
            ktp::build::ControlBuilder::SharedPtr control_builder_;
            ktp::build::MissionBuilder::SharedPtr mission_builder_;
            ktp::build::ObjectDetectBuilder::SharedPtr object_detect_builder_;
        public:
            explicit RequestBuilder(rclcpp::Node::SharedPtr node);
            virtual ~RequestBuilder();
        public:
            using SharedPtr = std::shared_ptr<RequestBuilder>;
        };
    }
}

#endif // REQUEST_BUILDER__HXX