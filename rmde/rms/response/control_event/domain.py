from enum import Enum
from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict
    

@dataclass
class ControlEventInfoLocation():
    xpos: float = 0.0
    ypos: float = 0.0
    heading: float = 0.0


@dataclass
class ControlEventInfoSubLocation():
    xpos: float = 0.0
    ypos: float = 0.0
    heading: float = 0.0
        
       
class ControlEventCdType(Enum):
    STOP: str = 'stop'
    ISOLATE: str = 'isolate'
    BROKEN: str = 'broken'
    

@dataclass
class ControlEventInfo():
    eventId: str = ''
    eventCd: str = ''
    eventSubCd: str = ''
    areaClsf: str = ''
    floor: str = ''
    batteryLevel: int = 0
    location: Dict = field(default_factory = empty_dict)
    subLocation: Dict = field(default_factory = empty_dict)
    

class ComInfoStatusType(Enum):
    CONNECTED: str = 'connected'
    DISCONNECTED: str = 'disconnected'
    
    
@dataclass
class ComInfo():
    status: str = ''
    robotIP: str = ''
    mqttIP: str = ''
    mqttPort: str = ''
    

@dataclass
class ControlEvent():
    header: Dict = field(default_factory = empty_dict)
    controlEventInfo: Dict = field(default_factory = empty_dict)
    comInfo: Dict = field(default_factory = empty_dict)
    

__all__ = ['rms_response_control_event_domain']