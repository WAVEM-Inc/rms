from enum import Enum
from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


class ControlCmdType(Enum):
    STOP: str = 'stop'
    GO: str = 'go'


@dataclass
class ControlInfo():
    controlId: str = ''
    controlCmd: str = ''


@dataclass
class ControlResult():
    status: str = ''
    startTime: str = ''
    endTime: str = ''


@dataclass
class TaskEventInfo():
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''
    jobGroup: str = ''
    jobKind: str = ''
    

@dataclass
class ControlEvent():
    header: Dict = field(default_factory = empty_dict)
    controlInfo: Dict = field(default_factory = empty_dict)
    controlResult: Dict = field(default_factory = empty_dict)
    taskEventInfo: Dict = field(default_factory = empty_dict)
    

__all__ = ['rms_response_control_event_domain']