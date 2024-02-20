#ifndef REQUEST_MANAGER__HXX
#define REQUEST_MANAGER__HXX

#include "builder/builder.hxx"

#define RBT_CONTROL_TOPIC "/ktp/data/control"
#define RBT_MISSION_TOPIC "/ktp/data/mission"
#define RBT_GRAPH_LIST_TOPIC "/ktp/data/graph_list"

namespace ktp
{
    namespace data
    {
        class RequestManager
        {
        private:
            rclcpp::Node::SharedPtr node_;

            ktp::build::MainBuilder::SharedPtr builder_;

            rclcpp::CallbackGroup::SharedPtr control_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Control>::SharedPtr control_subscription_;
            void control_subscription_cb(const ktp_data_msgs::msg::Control::SharedPtr control_cb);

            rclcpp::CallbackGroup::SharedPtr mission_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::Mission>::SharedPtr mission_subscription_;
            void mission_subscription_cb(const ktp_data_msgs::msg::Mission::SharedPtr mission_cb);

            rclcpp::CallbackGroup::SharedPtr graph_list_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::GraphList>::SharedPtr graph_list_subscription_;
            void graph_list_subscription_cb(const ktp_data_msgs::msg::GraphList::SharedPtr graph_list_cb);
        public:
            explicit RequestManager(rclcpp::Node::SharedPtr node);
            virtual ~RequestManager();
        public:
            using SharedPtr = std::shared_ptr<RequestManager>;
        };
    }
}

#endif