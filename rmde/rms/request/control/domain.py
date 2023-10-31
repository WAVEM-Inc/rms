
class ControlCmd():
    def __init__(
        self,
        ready: bool = None,
        move: bool = None,
        stop: bool = None
    ) -> None:
        self.ready: bool = ready
        self.move: bool = move
        self.stop: bool = stop
        

class Control():
    def __init__(
        self,
        header: dict = None,
        controlCmd: dict = None
    ) -> None:
        self.header: dict = header
        self.controlCmd: dict = controlCmd
        

__all__ = ['control']