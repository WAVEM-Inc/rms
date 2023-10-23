
class Header():
    def __init__(
        self,
        robotCorpId: str = None,
        workCorpId: str = None,
        workSiteId: str = None,
        robotId: str = None,
        robotType: str = None
    ):
        self.robotCorpId = robotCorpId
        self.workCorpId = workCorpId
        self.workSiteId = workSiteId
        self.robotId = robotId
        self.robotType = robotType