
class TaskInfo():
    def __init__(
        self,
        jobGroup: str = None,
        jobKind: str = None,
        taskStatus: str = None
    ) -> None:
        self.jobGroup = jobGroup
        self.jobKind = jobKind
        self.taskStatus = taskStatus