
class ComInfo():
    def __init__(
        self,
        status: str = None,
        robotIP: str = None,
        mqttIP: str = None,
        mqttPort: str = None
    ) -> None:
        self.status = status
        self.robotIP = robotIP
        self.mqttIP = mqttIP
        self.mqttPort = mqttPort