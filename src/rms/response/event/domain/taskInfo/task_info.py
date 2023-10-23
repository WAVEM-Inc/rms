
class TaskInfo():
    def __init__(self, jobPlanId=None, jobGroupId=None, jobOrderId=None, jobGroup=None, jobKind=None, jobResult=None) -> None:
        self.jobPlanId = jobPlanId
        self.jobGroupId = jobGroupId
        self.jobOrderId = jobOrderId
        self.jobKind = jobKind
        self.jobResult = jobResult