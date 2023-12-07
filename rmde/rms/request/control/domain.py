from enum import Enum
from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


class ControlCmdType(Enum):
    RESET: str = 'reset'
    GO: str = 'go'
    STOP: str = 'stop'


@dataclass
class ControlInfo():
    controlId: str = ''
    controlCmd: str = ''


@dataclass
class Control():
    header: Dict = field(default_factory=empty_dict)
    controlInfo: Dict = field(default_factory=empty_dict)


__all__ = ['rms_request_control_domain']
