
class ComInfo():
    def __init__(self, status=None, robotIP=None, mqttIP=None, mqttPort=None) -> None:
        self.status = status
        self.robotIP = robotIP
        self.mqttIP = mqttIP
        self.mqttPort = mqttPort