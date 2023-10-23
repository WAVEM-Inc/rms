
class EventLocation():
    def __init__(
        self,
        xpos: float = None,
        ypos: float = None,
        heading: float = None
    ) -> None:
        self.xpos = xpos
        self.ypos = ypos
        self.heading = heading