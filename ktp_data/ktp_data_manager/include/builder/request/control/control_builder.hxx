#ifndef CONTORL_BUILDER__HXX
#define CONTORL_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/control.hpp>

#include "model/model_enums.hxx"

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class ControlBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit ControlBuilder(rclcpp::Node::SharedPtr node);
            virtual ~ControlBuilder();
            ktp_data_msgs::msg::Control build_control();
        public:
            using SharedPtr = std::shared_ptr<ControlBuilder>;
        };
    }
}

#endif // CONTORL_BUILDER__HXX