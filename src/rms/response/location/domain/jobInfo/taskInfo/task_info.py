
class TaskInfo():
    def __init__(
        self,
        jobGroup: str = None,
        jobKind: str = None,
        taskStatus: str = None
    ) -> None:
        self.jobGroup: str = jobGroup
        self.jobKind: str = jobKind
        self.taskStatus: str = taskStatus
        

__all__ = ['task_info']