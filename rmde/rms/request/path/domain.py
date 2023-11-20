from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


@dataclass
class JobInfo():
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''
    jobGroup: str = ''
    jobKind: str = ''


@dataclass
class JobKindType():
    jobTargetId: str = ''
        

@dataclass
class JobPath():
    areaClsf: str = ''
    locationList: list = ''
    jobKindType: Dict = ''


@dataclass
class Path():
    header: Dict = field(default_factory = empty_dict)
    jobInfo: Dict = field(default_factory = empty_dict)
    jobPath: Dict = field(default_factory = empty_dict)
        
        
__all__ = ['rms_request_path_domain']