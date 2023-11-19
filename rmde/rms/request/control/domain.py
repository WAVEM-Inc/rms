from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


@dataclass
class ControlCmd():
    ready: bool = False
    move: bool = False
    stop: bool = False


@dataclass
class Control():
    header: Dict = field(default_factory = empty_dict)
    controlCmd: Dict = field(default_factory = empty_dict)
        

__all__ = ['rms_request_control_domain']