from dataclasses import dataclass

@dataclass
class Header():
    robotCorpId: str = ''
    workCorpId: str = ''
    workSiteId: str = ''
    robotId: str = ''
    robotType: str = ''


__all__ = ['rms_common_domain']