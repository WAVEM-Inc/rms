
class LastInfoLocation():
    def __init__(
        self,
        xpos: float = None,
        ypos: float = None,
        heading: float = None
    ) -> None:
        self.xpos: float = xpos
        self.ypos: float = ypos
        self.heading: float = heading
        

__all__ = ['last_info_location']