
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