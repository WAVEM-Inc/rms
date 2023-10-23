
class TaskInfo():
    def __init__(self, jobPlanId=None, jobGroupId=None, jobKind=None, jobResult=None) -> None:
        self.jobPlanId = jobPlanId
        self.jobGroupId = jobGroupId
        self.jobKind = jobKind
        self.jobResult = jobResult