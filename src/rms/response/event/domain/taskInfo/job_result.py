
class JobResult():
    def __init__(self, status=None, startTime=None, endTime=None, startBatteryLevel=None, endBatteryLevel=None, dist=None) -> None:
        self.status = status
        self.startTime = startTime
        self.startBatteryLevel = startBatteryLevel
        self.endBatteryLevel = endBatteryLevel
        self.dist = dist