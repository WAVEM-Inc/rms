
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
        

class LastInfoLocation():
    def __init__(
        self,
        xpos: float = None,
        ypos: float = None,
        heading: float = None
    ) -> None:
        self.xpos: float = xpos
        self.ypos: float = ypos
        self.heading: float = heading
        

class LastInfo():
    def __init__(
        self,
        location: dict = None,
        areaClsf: str = None,
        floor: str = None,
        batteryLevel: int = None,
        velocity: float = None,
        totalDist: int = None
    ) -> None:
        self.location: dict = location
        self.areaClsf: str = areaClsf
        self.floor: str = floor
        self.batteryLevel: int = batteryLevel
        self.velocity: int = velocity
        self.totalDist: int = totalDist
        

class Location():
    def __init__(
        self,
        header: dict = None,
        jobInfo: dict = None,
        lastInfo: dict = None
    ) -> None:
        self.header: dict = header
        self.jobInfo: dict = jobInfo
        self.lastInfo: dict = lastInfo
        
        
__all__ = ['location']