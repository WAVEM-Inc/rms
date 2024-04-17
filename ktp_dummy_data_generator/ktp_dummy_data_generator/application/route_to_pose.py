import time;
from rclpy.node import Node;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from rclpy.action import ActionServer;
from rclpy.action.server import ServerGoalHandle;
from rclpy.action.server import GoalResponse;
from rclpy.action.server import default_goal_callback;
from rclpy.action.server import default_handle_accepted_callback;
from rclpy.action.server import default_cancel_callback;
from rclpy.publisher import Publisher;
from rclpy.qos import qos_profile_system_default;
from rclpy.qos import qos_profile_services_default;
from rclpy.qos import qos_profile_action_status_default;
from rclpy.qos import QoSProfile;
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup;
from route_msgs.action import RouteToPose;
from obstacle_msgs.msg import Status;
from typing import Any;
from rclpy.timer import Timer;

ROUTE_TO_POSE_ACTION_NAME: str = "/route_to_pose";
OBSTACLE_EVENT_TOPIC_NAME: str = "/drive/obstacle/event";


class DummyRouteToPose:

    def __init__(self, node: Node) -> None:
        self.__node: Node = node;
        self.__log: RcutilsLogger = self.__node.get_logger();

        route_to_pose_action_server_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__route_to_pose_action_server: ActionServer = ActionServer(
            node=self.__node,
            action_name=ROUTE_TO_POSE_ACTION_NAME,
            action_type=RouteToPose,
            callback_group=route_to_pose_action_server_cb_group,
            execute_callback=self.route_to_pose_execute_cb,
            goal_callback=default_goal_callback,
            handle_accepted_callback=default_handle_accepted_callback,
            cancel_callback=default_cancel_callback,
            result_service_qos_profile=qos_profile_services_default,
            cancel_service_qos_profile=qos_profile_services_default,
            feedback_pub_qos_profile=QoSProfile(depth=10),
            status_pub_qos_profile=qos_profile_action_status_default
        );

        obstacle_event_publisher_cb_group: MutuallyExclusiveCallbackGroup = MutuallyExclusiveCallbackGroup();
        self.__obstacle_event_publisher: Publisher = self.__node.create_publisher(
            topic=OBSTACLE_EVENT_TOPIC_NAME,
            msg_type=Status,
            qos_profile=qos_profile_system_default,
            callback_group=obstacle_event_publisher_cb_group
        );

    def route_to_pose_execute_cb(self, goal_handle: ServerGoalHandle) -> RouteToPose.Result:
        feedback: RouteToPose.Feedback = RouteToPose.Feedback();
        feedback.status_code = 1001;

        goal_handle.publish_feedback(feedback=feedback);
        time.sleep(0.5);
        
        goal_handle.succeed();
        time.sleep(0.5);
        
        result: RouteToPose.Result = RouteToPose.Result();
        result.result = 1001;

        return result;


__all__ = ["DummyRouteToPose"];