#ifndef CONTROL_MANAGER__HXX
#define CONTROL_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/control.hpp>

#define DEFAULT_QOS 10

#define CONTROL_FROM_ITF_TOPIC "/rms/ktp/data/control"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class ControlManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr control_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Control>::SharedPtr control_from_itf_subscription_;
            void control_from_itf_subscription_cb(const ktp_data_msgs::msg::Control::SharedPtr control_cb);

        public:
            explicit ControlManager(rclcpp::Node::SharedPtr node);
            virtual ~ControlManager();
        public:
            using SharedPtr = std::shared_ptr<ControlManager>;
        };
    }

}

#endif // CONTROL_MANAGER__HXX