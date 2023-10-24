
class SetInfo():
    def __init__(
        self,
        robotType: str = None,
        mqttIP: str = None,
        mqttPort: str = None,
        robotCorpId: str = None,
        workCorpId: str = None,
        workSiteId: str = None,
        batteryEvent: str = None
    ) -> None:
        self.robotType: str = robotType
        self.mqttIP: str = mqttIP
        self.mqttPort: str = mqttPort
        self.robotCorpId: str = robotCorpId
        self.workCorpId: str = workCorpId
        self.workSiteId: str = workSiteId
        self.batteryEvent: str = batteryEvent
        

__all__ = ['set_info']