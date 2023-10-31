from enum import Enum

class AreaCLSFType(Enum):
    INDOOR: str = 'indoor'
    OUTDOOR: str = 'outdoor'
    
    
class JobGroupType(Enum):
    SUPPLY: str = 'supply'
    RECOVERY: str = 'recovery'
    MOVE: str = 'move'
    CHARGE: str = 'charge'
    WAIT: str = 'wait'
    

class JobKindType(Enum):
    LOAD: str = 'load'
    UNLOAD: str = 'unload'
    CHARGE: str = 'charge'
    WAIT: str = 'wait'
    MOVE: str = 'move'
    

class JobResultStatusType(Enum):
    SUCCESS: str = 'success'
    FAIL: str = 'fail'
    

class TaskStatusType(Enum):
    ASSIGNED: str = 'assigned'
    

class RobotType(Enum):
    AMR: str = 'AMR'