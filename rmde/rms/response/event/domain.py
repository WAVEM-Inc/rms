from enum import Enum
from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


class ComInfoStatusType(Enum):
    CONNECTED: str = 'connected'
    DISCONNECTED: str = 'disconnected'
    

@dataclass
class EventInfoLocation():
    xpos: float = 0.0
    ypos: float = 0.0
    heading: float = 0.0


@dataclass
class EventInfoSubLocation():
    xpos: float = 0.0
    ypos: float = 0.0
        
       
class EventCdType(Enum):
    STOP: str = 'stop'
    ISOLATE: str = 'isolate'
    BROKEN: str = 'broken'
    

@dataclass
class EventInfo():
    eventId: str = ''
    eventCd: str = ''
    eventSubCd: str = ''
    areaClsf: str = ''
    floor: str = ''
    batteryLevel: int = 0
    location: Dict = field(default_factory = empty_dict)
    subLocation: Dict = field(default_factory = empty_dict)
        

@dataclass
class JobResult():
    status: str = ''
    startTime: str = ''
    endTime: str = ''
    startBatteryLevel: int = 0
    endBatteryLevel: int = 0
    dist: int = 0
        

@dataclass
class TaskInfo():
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''
    jobGroup: str = ''
    jobKind: str = ''
    jobResult: Dict = field(default_factory = empty_dict)
        

@dataclass       
class ComInfo():
    status: str = ''
    robotIP: str = ''
    mqttIP: str = ''
    mqttPort: str = ''


@dataclass
class Event():
    header: Dict = field(default_factory = empty_dict)
    taskInfo: Dict = field(default_factory = empty_dict)
    eventInfo: Dict = field(default_factory = empty_dict)
    comInfo: Dict = field(default_factory = empty_dict)
        

__all__ = ['rms_response_event_domain']