#ifndef LIDAR_SIGNAL_MANAGER__HXX
#define LIDAR_SIGNAL_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ktp_data_msgs/msg/li_dar_signal.hpp>

#include "utils/utils.hxx"

#define DEFAULT_QOS 10
#define OBSTACLE_COOPERATIVE_FROM_STATE_OBSTACLE_TOPIC "/drive/obstacle/cooperative"
#define OBSTACLE_COOPERATIVE_START "START"
#define OBSTACLE_COOPERATIVE_STOP "STOP"

#define LIDAR_SIGNAL_TO_ITF_TOPIC "/rms/ktp/data/lidar_signal"

using std::placeholders::_1;


namespace ktp
{
    namespace data
    {
        class LiDARSignalManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr obstacle_cooperative_subscription_cb_group_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_cooperative_subscription_;
            void obstacle_cooperative_subscription_cb(const std_msgs::msg::String::SharedPtr obstacle_cooperative_cb);

            rclcpp::CallbackGroup::SharedPtr lidar_signal_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::LiDARSignal>::SharedPtr lidar_signal_to_itf_publisher_;
            void lidar_signal_to_itf_publish(std::string obstacle_cooperative_flag);

        public:
            explicit LiDARSignalManager(rclcpp::Node::SharedPtr node);
            virtual ~LiDARSignalManager();
            
        public:
            using SharedPtr = std::shared_ptr<LiDARSignalManager>;
        };
    }
}

#endif // LIDAR_SIGNAL_MANAGER__HXX