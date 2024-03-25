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

#define PATH_GRAPH_GRAPH_SERVICE_NAME "/path_graph_msgs/graph"
#define GRAPH_LIST_TO_ITF_TOPIC "/rms/ktp/data/graph_list"

#define DEV_ID "KECDSEMITB001"

namespace ktp
{
    namespace data
    {
        class GraphListManager final
        {
        private:
            rclcpp::Node::SharedPtr node_;

            rclcpp::CallbackGroup::SharedPtr path_graph_graph_service_client_cb_group_;
            rclcpp::Client<path_graph_msgs::srv::Graph>::SharedPtr path_graph_graph_service_client_;

            rclcpp::CallbackGroup::SharedPtr graph_list_to_itf_publisher_cb_group_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr graph_list_to_itf_publisher_;
          void graph_list_publish(std::string response_graph_list);

        public:
            explicit GraphListManager(rclcpp::Node::SharedPtr node);
            virtual ~GraphListManager();
            void path_graph_graph_request();

        public:
            using SharedPtr = std::shared_ptr<GraphListManager>;
        };
    }
}

#endif // GRAPH_LIST_MANAGER__HXX