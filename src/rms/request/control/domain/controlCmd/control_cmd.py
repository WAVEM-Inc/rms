
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
        

__all__ = ['control_cmd']