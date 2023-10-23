from rms.response.event.domain.taskInfo.jobResult.job_result import JobResult

class TaskInfo():
    def __init__(
        self,
        jobPlanId: str = None,
        jobGroupId: str = None,
        jobOrderId: str = None,
        jobGroup: str = None,
        jobKind: str = None,
        jobResult: dict = None
    ) -> None:
        self.jobPlanId = jobPlanId
        self.jobGroupId = jobGroupId
        self.jobOrderId = jobOrderId
        self.jobGroup = jobGroup
        self.jobKind = jobKind
        self.jobResult = jobResult