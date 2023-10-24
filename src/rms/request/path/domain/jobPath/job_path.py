
class JobPath():
    def __init__(
        self,
        areaClsf: str = None,
        locationList: list = None,
        jobKindType: dict = None
    ) -> None:
        self.areaClsf: str = areaClsf
        self.locationList: list = locationList
        self.jobKindType: dict = jobKindType
        

__all__ = ['job_path']