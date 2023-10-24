
class JobInfo():
    def __init__(
        self,
        jobPlanId: str = None,
        jobGroupId: str = None,
        jobOrderId: str = None,
        taskInfo: dict = None
    ) -> None:
        self.jobPlanId: str = jobPlanId
        self.jobGroupId: str = jobGroupId
        self.jobOrderId: str = jobOrderId
        self.taskInfo: dict = taskInfo
        
        
__all__ = ['job_info']