
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
        

__all__ = ['last_info']