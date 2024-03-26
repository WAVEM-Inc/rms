#ifndef DETECTED_OBJECT_MANAGER__HXX
#define DETECTED_OBJECT_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/detected_object.hpp>

#define DEFAULT_QOS 10
#define DETECTED_OBJECT_FROM_ITF_TOPIC "/rms/ktp/itf/detected_object"
#define OBJECT_DETECT_TO_STATE_OBSTACLE_TOPIC "/drive/object_detect"

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

            rclcpp::CallbackGroup::SharedPtr detected_object_from_itf_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::DetectedObject>::SharedPtr detected_object_from_itf_subscription_;
            void detected_object_from_itf_subscription_cb(const ktp_data_msgs::msg::DetectedObject::SharedPtr detected_object_cb);

            rclcpp::CallbackGroup::SharedPtr object_detect_to_state_obstacle_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::DetectedObject>::SharedPtr object_detect_to_state_obstacle_publisher_;
            void object_detect_to_state_obstacle_publish(const ktp_data_msgs::msg::DetectedObject::SharedPtr detected_object);

        public:
            explicit DetectedObjectManager(rclcpp::Node::SharedPtr node);
            virtual ~DetectedObjectManager();

        public:
            using SharedPtr = std::shared_ptr<DetectedObjectManager>;
        };
    }

}

#endif // DETECTED_OBJECT_MANAGER__HXX