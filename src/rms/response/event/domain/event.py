from rms.common.domain.header import Header
from rms.response.event.domain.taskInfo.task_info import TaskInfo
from rms.response.event.domain.eventInfo.event_info import EventInfo
from rms.response.event.domain.comInfo.com_info import ComInfo

class Event():
    def __init__(
        self,
        header: dict = None,
        taskInfo: dict = None,
        eventInfo: dict = None,
        comInfo: dict = None
    ) -> None:
        self.header = header
        self.taskInfo = taskInfo
        self.eventInfo = eventInfo
        self.comInfo = comInfo