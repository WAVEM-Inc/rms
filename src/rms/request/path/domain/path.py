
class Path():
    def __init__(
        self,
        header: dict = None,
        jobInfo: dict = None,
        jobPath: dict = None
    ) -> None:
        self.header: dict = header
        self.jobInfo: dict = jobInfo
        self.jobPath: dict = jobPath
        
        
__all__ = ['path']