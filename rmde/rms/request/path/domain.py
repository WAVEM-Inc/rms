
class JobInfo():
    def __init__(
        self,
        jobPlanId: str = None,
        jobGroupId: str = None,
        jobOrderId: str = None,
        jobGroup: str = None,
        jobKind: str = None
    ) -> None:
        self.jobPlanId: str = jobPlanId
        self.jobGroupId: str = jobGroupId
        self.jobOrderId: str = jobOrderId
        self.jobGroup: str = jobGroup
        self.jobKind: str = jobKind


class JobKindType():
    def __init__(
        self,
        jobTargetId: str = None
    ) -> None:
        self.jobTargetId: str = jobTargetId
        

class JobPath():
    def __init__(
        self,
        areaClsf: str = None,
        locationList: list = None,
        jobKindType: dict = None
    ) -> None:
        self.areaClsf: str = areaClsf
        self.locationList: list = locationList
        self.jobKindType: dict = jobKindType
        

class Path():
    def __init__(
        self,
        header: dict = None,
        jobInfo: dict = None,
        jobPath: dict = None
    ) -> None:
        self.header: dict = header
        self.jobInfo: dict = jobInfo
        self.jobPath: dict = jobPath
        
        
__all__ = ['path']