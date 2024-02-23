#ifndef OBJECT_DETECT_MANAGER__HXX
#define OBJECT_DETECT_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/object_detect.hpp>

#define DEFAULT_QOS 10

#define OBJECT_DETECT_FROM_ITF_TOPIC "/rms/ktp/data/object_detect"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace data
    {
        class ObjectDetectManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr object_detect_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::ObjectDetect>::SharedPtr object_detect_from_itf_subscription_;
            void object_detect_from_itf_subscription_cb(const ktp_data_msgs::msg::ObjectDetect::SharedPtr object_detect_cb);

        public:
            explicit ObjectDetectManager(rclcpp::Node::SharedPtr node);
            virtual ~ObjectDetectManager();

        public:
            using SharedPtr = std::shared_ptr<ObjectDetectManager>;
        };
    }

}

#endif // OBJECT_DETECT_MANAGER__HXX