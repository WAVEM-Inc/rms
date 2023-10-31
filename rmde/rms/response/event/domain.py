from enum import Enum

class ComInfoStatusType(Enum):
    CONNECTED: str = 'connected'
    DISCONNECTED: str = 'disconnected'
    

class EventInfoLocation():
    def __init__(
        self,
        xpos: float = None,
        ypos: float = None,
        heading: float = None
    ) -> None:
        self.xpos = xpos
        self.ypos = ypos
        self.heading = heading

        
class EventCdType(Enum):
    STOP: str = "stop"
    ISOLATE: str = "isolate"
    BROKEN: str = "broken"
    

class EventInfo():
    def __init__(
        self, 
        eventId: str = None,
        eventCd: str = None,
        eventSubCd: str = None,
        areaClsf: str = None,
        floor: str = None,
        batteryLevel: int = None,
        location: dict = None
    ) -> None:
        self.eventId = eventId
        self.eventCd = eventCd
        self.eventSubCd = eventSubCd
        self.areaClsf = areaClsf
        self.floor = floor
        self.batteryLevel = batteryLevel
        self.location = location
        


class JobResult():
    
    def __init__(
        self,
        status: str = None, 
        startTime: str = None, 
        endTime: str = None, 
        startBatteryLevel: int = None,
        endBatteryLevel: int = None,
        dist: int = None
    ) -> None:
        self.status = status
        self.startTime = startTime
        self.endTime = endTime
        self.startBatteryLevel = startBatteryLevel
        self.endBatteryLevel = endBatteryLevel
        self.dist = dist
        

class TaskInfo():
    def __init__(
        self,
        jobPlanId: str = None,
        jobGroupId: str = None,
        jobOrderId: str = None,
        jobGroup: str = None,
        jobKind: str = None,
        jobResult: dict = None
    ) -> None:
        self.jobPlanId = jobPlanId
        self.jobGroupId = jobGroupId
        self.jobOrderId = jobOrderId
        self.jobGroup = jobGroup
        self.jobKind = jobKind
        self.jobResult = jobResult
        
        
class ComInfo():
    def __init__(
        self,
        status: str = None,
        robotIP: str = None,
        mqttIP: str = None,
        mqttPort: str = None
    ) -> None:
        self.status: str = status
        self.robotIP: str = robotIP
        self.mqttIP: str = mqttIP
        self.mqttPort: str = mqttPort


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
        

__all__ = ['event']