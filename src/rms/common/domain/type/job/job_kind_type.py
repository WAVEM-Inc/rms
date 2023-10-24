from enum import Enum

class JobKindType(Enum):
    LOAD: str = "load"
    UNLOAD: str = "unload"
    CHARGE: str = "charge"
    WAIT: str = "wait"
    MOVE: str = "move"