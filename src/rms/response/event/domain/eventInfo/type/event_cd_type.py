from enum import Enum

class EventCdType(Enum):
    STOP: str = "stop"
    ISOLATE: str = "isolate"
    BROKEN: str = "broken"