from dataclasses import dataclass

@dataclass
class Header():
    robotCorpId: str = ''
    workCorpId: str = ''
    workSiteId: str = ''
    robotId: str = ''
    robotType: str = ''


@dataclass
class Job():
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''


__all__ = ['rms_common_domain']