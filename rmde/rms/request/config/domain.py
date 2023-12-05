from dataclasses import dataclass, field
from typing import Dict
from ...common.service import empty_dict


@dataclass
class SetInfo():
    robotType: str = '',
    mqttIP: str = '',
    mqttPort: str = '',
    robotCorpId: str = '',
    robotId: str = '',
    workCorpId: str = '',
    workSiteId: str = '',
    batteryEvent: str = ''


@dataclass
class Config:
    header: Dict = field(default_factory=empty_dict)
    setInfo: Dict = field(default_factory=empty_dict)


__all__ = ['rms_request_config_domain']
