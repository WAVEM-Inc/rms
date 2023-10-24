
class JobResult():
    
    def __init__(
        self,
        status: str = None, 
        startTime: str = None, 
        endTime: str = None, 
        startBatteryLevel: int = None,
        endBatteryLevel: int = None,
        dist: int = None
    ) -> None:
        self.status = status
        self.startTime = startTime
        self.endTime = endTime
        self.startBatteryLevel = startBatteryLevel
        self.endBatteryLevel = endBatteryLevel
        self.dist = dist
        

__all__ = ['job_result']