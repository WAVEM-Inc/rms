from enum import Enum
from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


@dataclass
class JobResult():
    status: str = ''
    startTime: str = ''
    endTime: str = ''
    startBatteryLevel: int = 0
    endBatteryLevel: int = 0
    dist: int = 0


@dataclass
class TaskEventInfo():
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''
    jobGroup: str = ''
    jobKind: str = ''
    jobResult: Dict = field(default_factory=empty_dict)


@dataclass
class TaskEvent():
    header: Dict = field(default_factory=empty_dict)
    taskEventInfo: Dict = field(default_factory=empty_dict)


__all__ = ['rms_response_task_event_domain']
