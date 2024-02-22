#ifndef REQUEST_MANAGER__HXX
#define REQUEST_MANAGER__HXX

#include "builder/builder.hxx"

#include <ktp_data_msgs/msg/control.hpp>

#include <ktp_data_msgs/msg/mission.hpp>
#include <ktp_data_msgs/msg/mission_task.hpp>
#include <ktp_data_msgs/msg/mission_task_data.hpp>

#include <ktp_data_msgs/msg/object_detect.hpp>

#define CONTROL_FROM_ITF_TOPIC "/ktp/data/control"
#define MISSION_FROM_ITF_TOPIC "/ktp/data/mission"
#define OBJECT_DETECT_FROM_ITF_TOPIC "/ktp/data/object_detect"

namespace ktp
{
    namespace data
    {
        class RequestManager
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::build::MainBuilder::SharedPtr builder_;

            rclcpp::CallbackGroup::SharedPtr control_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Control>::SharedPtr control_from_itf_subscription_;
            void control_from_itf_subscription_cb(const ktp_data_msgs::msg::Control::SharedPtr control_cb);

            rclcpp::CallbackGroup::SharedPtr mission_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Mission>::SharedPtr mission_from_itf_subscription_;
            void mission_from_itf_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb);

            rclcpp::CallbackGroup::SharedPtr object_detect_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::ObjectDetect>::SharedPtr object_detect_from_itf_subscription_;
            void object_detect_from_itf_subscription_cb(const ktp_data_msgs::msg::ObjectDetect::SharedPtr object_detect_cb);

        public:
            explicit RequestManager(rclcpp::Node::SharedPtr node);
            virtual ~RequestManager();

        public:
            using SharedPtr = std::shared_ptr<RequestManager>;
        };
    }
}

#endif