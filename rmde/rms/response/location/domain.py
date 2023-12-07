from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


@dataclass
class TaskInfo():
    jobGroup: str = ''
    jobKind: str = ''
    taskStatus: str = ''


@dataclass
class JobInfo():
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''
    taskInfo: Dict = field(default_factory=empty_dict)


@dataclass
class LastInfoLocation():
    xpos: float = 0.0
    ypos: float = 0.0
    heading: float = 0.0


@dataclass
class LastInfoSubLocation():
    xpos: float = 0.0
    ypos: float = 0.0
    heading: float = 0.0


@dataclass
class LastInfo():
    location: Dict = field(default_factory=empty_dict)
    subLocation: Dict = field(default_factory=empty_dict)
    areaClsf: str = ''
    floor: str = ''
    batteryLevel: int = 0
    velocity: float = 0.0
    totalDist: int = 0


@dataclass
class Location():
    header: Dict = field(default_factory=empty_dict)
    jobInfo: Dict = field(default_factory=empty_dict)
    lastInfo: Dict = field(default_factory=empty_dict)


__all__ = ['rms_response_location_domain']
