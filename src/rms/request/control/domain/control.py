
class Control():
    def __init__(
        self,
        header: dict = None,
        controlCmd: dict = None
    ) -> None:
        self.header: dict = header
        self.controlCmd: dict = controlCmd
        

__all__ = ['control']