
class ComInfo():
    def __init__(
        self,
        status: str = None,
        robotIP: str = None,
        mqttIP: str = None,
        mqttPort: str = None
    ) -> None:
        self.status: str = status
        self.robotIP: str = robotIP
        self.mqttIP: str = mqttIP
        self.mqttPort: str = mqttPort
        

__all__ = ['com_info']