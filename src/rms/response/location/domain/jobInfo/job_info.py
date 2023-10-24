
class JobInfo():
    def __init__(
        self,
        jobPlanId: str = None,
        jobGroupId: str = None,
        jobOrderId: str = None,
        taskInfo: dict = None
    ) -> None:
        self.jobPlanId = jobPlanId
        self.jobGroupId = jobGroupId
        self.jobOrderId = jobOrderId
        self.taskInfo = taskInfo