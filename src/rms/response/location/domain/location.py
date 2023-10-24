
class Location():
    def __init__(
        self,
        header: dict = None,
        jobInfo: dict = None,
        lastInfo: dict = None
    ) -> None:
        self.header = header
        self.jobInfo = jobInfo
        self.lastInfo = lastInfo