
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
        

__all__ = ['job_info']