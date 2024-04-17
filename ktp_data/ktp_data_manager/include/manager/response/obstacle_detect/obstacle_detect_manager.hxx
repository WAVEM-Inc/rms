#ifndef OBSTACLE_DETECT_MANAGER_HXX
#define OBSTACLE_DETECT_MANAGER_HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/obstacle_detect.hpp>

#define DEFAULT_QOS 0

#define OBSTACLE_DETECT_FROM_TASK_CTRL_TOPIC "/rms/ktp/task/notify/obstacle_detect"
#define OBSTACLE_DETECT_TO_ITF_TOPIC "/rms/ktp/data/obstacle_detect"

using std::placeholders::_1;

namespace ktp
{
    namespace data
    {
        class ObstacleDetectManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr obstacle_detect_from_task_ctrl_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::ObstacleDetect>::SharedPtr obstacle_detect_from_task_ctrl_subscription_;
            void obstacle_detect_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::ObstacleDetect::SharedPtr obstacle_detect_cb);

            rclcpp::CallbackGroup::SharedPtr obstacle_detect_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::ObstacleDetect>::SharedPtr obstacle_detect_to_itf_publisher_;

        public:
            explicit ObstacleDetectManager(rclcpp::Node::SharedPtr node);
            virtual ~ObstacleDetectManager();

        public:
            using SharedPtr = std::shared_ptr<ObstacleDetectManager>;
        };
    }
}

#endif // OBSTACLE_DETECT_MANAGER_HXX
