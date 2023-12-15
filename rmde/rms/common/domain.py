from dataclasses import dataclass


@dataclass
class Header():
    robotCorpId: str = ''
    workCorpId: str = ''
    workSiteId: str = ''
    robotId: str = ''
    robotType: str = ''
    topicUid: str = ''


@dataclass
class JobUUID():
    jobGroup: str = ''
    jobKind: str = ''
    jobPlanId: str = ''
    jobGroupId: str = ''
    jobOrderId: str = ''


__all__ = ['rms_common_domain']
