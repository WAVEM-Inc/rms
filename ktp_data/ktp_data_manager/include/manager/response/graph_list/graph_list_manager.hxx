#ifndef GRAPH_LIST_MANAGER__HXX
#define GRAPH_LIST_MANAGER__HXX

#include <rclcpp/rclcpp.hpp>

#include <ktp_data_msgs/msg/graph_list.hpp>
#include <ktp_data_msgs/msg/graph.hpp>
#include <ktp_data_msgs/msg/graph_node_list.hpp>
#include <ktp_data_msgs/msg/graph_edge_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_vertices.hpp>

#define DEFAULT_QOS 10

#define GRAPH_LIST_TO_ITF_TOPIC "/rms/ktp/data/graph_list"

namespace ktp
{
    namespace data
    {
        class GraphListManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

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