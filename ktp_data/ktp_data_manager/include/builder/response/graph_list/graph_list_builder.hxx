#ifndef GRAPH_LIST_BUILDER__HXX
#define GRAPH_LIST_BUILDER__HXX

#include <rclcpp/rclcpp.hpp>
#include <ktp_data_msgs/msg/graph_list.hpp>
#include <ktp_data_msgs/msg/graph.hpp>
#include <ktp_data_msgs/msg/graph_node_list.hpp>
#include <ktp_data_msgs/msg/graph_edge_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_list.hpp>
#include <ktp_data_msgs/msg/graph_zone_vertices.hpp>

#include "model/model_enums.hxx"

#define DEFAULT_QOS 10

using std::placeholders::_1;
using std::placeholders::_2;

namespace ktp
{
    namespace build
    {
        class GraphListBuilder final
        {
        private:
            rclcpp::Node::SharedPtr node_;
        public:
            explicit GraphListBuilder(rclcpp::Node::SharedPtr node);
            virtual ~GraphListBuilder();
        };
    }
}

#endif