from enum import Enum
from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


class CmdResultTopicKindType(Enum):
    PATH: str = 'path'
    CONFIG: str = 'config'
    CONTROL: str = 'control'


@dataclass
class CmdResult():
    status: str = ''
    startTime: str = ''
    topicKind: str = ''


@dataclass
class CmdResponse():
    header: Dict = field(default_factory = empty_dict)
    cmdResult: Dict = field(default_factory = empty_dict)


__all__ = ['rms_response_cmd_repsonse_domain']