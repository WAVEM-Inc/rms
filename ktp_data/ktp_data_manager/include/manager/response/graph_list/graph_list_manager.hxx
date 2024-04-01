#ifndef GRAPH_LIST_MANAGER__HXX
#define GRAPH_LIST_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <ktp_data_msgs/msg/graph_list.hpp>
#include <ktp_data_msgs/msg/graph.hpp>
#include <ktp_data_msgs/msg/graph_node_list.hpp>
#include <ktp_data_msgs/msg/graph_edge_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_vertices.hpp>

#include <path_graph_msgs/srv/graph.hpp>

#include "utils/utils.hxx"

#define DEFAULT_QOS 10

#define GRAPH_LIST_TO_ITF_TOPIC "/rms/ktp/data/graph_list"
#define GRAPH_LIST_FROM_TASK_CTRL_TOPIC "/rms/ktp/task/notify/graph_list"

#define DEV_ID "KECDSEMITB001"

using std::placeholders::_1;

namespace ktp
{
    namespace data
    {
        class GraphListManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr graph_list_from_task_ctrl_subscription_cb_group_;
            rclcpp::Subscription<ktp_data_msgs::msg::GraphList>::SharedPtr graph_list_from_task_ctrl_subscription_;
            void graph_list_from_task_ctrl_subscription_cb(const ktp_data_msgs::msg::GraphList::SharedPtr graph_list_cb);

            rclcpp::CallbackGroup::SharedPtr graph_list_to_itf_publisher_cb_group_;
            rclcpp::Publisher<ktp_data_msgs::msg::GraphList>::SharedPtr graph_list_to_itf_publisher_;

        public:
            explicit GraphListManager(rclcpp::Node::SharedPtr node);
            virtual ~GraphListManager();

        public:
            using SharedPtr = std::shared_ptr<GraphListManager>;
        };
    }
}

#endif // GRAPH_LIST_MANAGER__HXX