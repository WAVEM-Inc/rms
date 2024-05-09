from rclpy.node import Node;
from rclpy.subscription import Subscription;
from rclpy.qos import qos_profile_system_default;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from ktp_data_msgs.msg import GraphList;
from ktp_data_manager.application.graph_list import GraphListService;


GRAPH_LIST_FROM_TASK_CTRL_TOPIC: str = "/rms/ktp/task/notify/graph_list";


class GraphListController:
    
    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__graph_service: GraphListService = GraphListService(node=self.__node);
        
        graph_list_from_task_ctrl_subscription_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__graph_list_from_task_ctrl_subscription: Subscription = self.__node.create_subscription(
            topic=GRAPH_LIST_FROM_TASK_CTRL_TOPIC,
            msg_type=GraphList,
            qos_profile=qos_profile_system_default,
            callback_group=graph_list_from_task_ctrl_subscription_cb_group,
            callback=self.graph_list_from_task_ctrl_subscription_cb
        );
                
    def graph_list_from_task_ctrl_subscription_cb(self, graph_list_cb: GraphList) -> None:
        self.__graph_service.graph_list_response(graph_list=graph_list_cb);


__all__: list[str] = ["GraphListController"];