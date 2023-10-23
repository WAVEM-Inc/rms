from rms.common.domain.header import Header
from rms.response.event.domain.taskInfo.task_info import TaskInfo
from rms.response.event.domain.eventInfo.event_info import EventInfo
from rms.response.event.domain.comInfo.com_info import ComInfo
from mqtt import broker

class EventResponseHandler():
    header: Header = Header()
    taskInfo: TaskInfo = TaskInfo()
    eventInfo: EventInfo = EventInfo()
    comInfo: ComInfo = ComInfo()
    
    def __init__(self, mqtt_broker: broker.mqtt_broker) -> None:
        self.mqtt_broker = mqtt_broker
        self.header = Header('1')
    
    def response_to_uvc(self) -> None:
        self.mqtt_broker.client.publish('hubilon/atcplus/ros/event', 'hihi')