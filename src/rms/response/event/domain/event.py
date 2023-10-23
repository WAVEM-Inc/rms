
class Event():
    def __init__(self, header=None, taskInfo=None, eventInfo=None, comInfo=None) -> None:
        self.header = header
        self.taskInfo = taskInfo
        self.eventInfo = eventInfo
        self.comInfo = comInfo