from enum import Enum

class JobGroupType(Enum):
    SUPPLY: str = "supply"
    RECOVERY: str = "recovery"
    MOVE: str = "move"
    CHARGE: str = "charge"
    WAIT: str = "wait"