
class Event():
    def __init__(
        self,
        header: dict = None,
        taskInfo: dict = None,
        eventInfo: dict = None,
        comInfo: dict = None
    ) -> None:
        self.header = header
        self.taskInfo = taskInfo
        self.eventInfo = eventInfo
        self.comInfo = comInfo
        

__all__ = ['event']