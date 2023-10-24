
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
        self.location = location
        self.areaClsf = areaClsf
        self.floor = floor
        self.batteryLevel = batteryLevel
        self.velocity = velocity
        self.totalDist = totalDist