from rclpy.node import Node;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import GraphList;


GRAPH_LIST_TO_ITF_TOPIC: str = "/rms/ktp/data/graph_list";


class GraphListService:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        
        graph_list_to_itf_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_to_itf_publisher: Publisher = self.__node.create_publisher(
            topic=GRAPH_LIST_TO_ITF_TOPIC,
            msg_type=GraphList,
            qos_profile=qos_profile_system_default,
            callback_group=graph_list_to_itf_publisher_cb_group
        );
        
    def graph_list_response(self, graph_list: GraphList) -> None:
        self.__graph_list_to_itf_publisher.publish(msg=graph_list);
        
        
__all__: list[str] = ["GraphListService"];