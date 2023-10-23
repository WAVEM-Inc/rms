import rclpy
import json

from rclpy.node import Node
from mqtt import broker
from rms.common.domain.header import Header
from rms.response.event.domain.taskInfo.task_info import TaskInfo
from rms.response.event.domain.eventInfo.event_info import EventInfo
from rms.response.event.domain.comInfo.com_info import ComInfo
from rms.response.event.domain.taskInfo.job_result import JobResult
from rms.common.application.uuid_service import UUIDService
from rms.common.application.time_service import TimeService

from typing import Dict

class EventResponseHandler():
    header: Header = Header()
    taskInfo: TaskInfo = TaskInfo()
    eventInfo: EventInfo = EventInfo()
    comInfo: ComInfo = ComInfo()
    
    
    def __init__(self, rclpy_node: Node, mqtt_broker: broker.mqtt_broker) -> None:
        self.rclpy_node = rclpy_node
        self.mqtt_broker = mqtt_broker
        self.uuid_service = UUIDService()
        self.time_service = TimeService()
        
        self.__build_header__()
        self.__build__task_info__()
    
    
    def __build_header__(self) -> None:
        self.header = Header(
            robotCorpId = 'rco0000001',
            workCorpId = 'wco0000001',
            workSiteId = 'wst0000001',
            robotId = 'rbt0000001',
            robotType ='AMR'
        )
        

    def __build__task_info__(self) -> None:
        job_result : JobResult = JobResult(
            status = "success",
            startTime = self.time_service.get_current_datetime(),
            endTime = self.time_service.get_current_datetime(),
            startBatteryLevel = 50,
            endBatteryLevel = 50,
            dist = 300
        )
        
        self.taskInfo = TaskInfo(
            jobPlanId = self.uuid_service.generate_uuid(),
            jobGroupId = self.uuid_service.generate_uuid(),
            jobOrderId = self.uuid_service.generate_uuid(),
            jobGroup = 'supply',
            jobKind = 'move',
            jobResult = job_result
        )
    
    
    def response_to_uvc(self) -> None:
        self.mqtt_broker.client.publish('hubilon/atcplus/ros/event', json.dumps(self.taskInfo.__dict__))