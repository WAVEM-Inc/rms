from enum import Enum

class ComInfoStatusType(Enum):
    CONNECTED: str = 'connected'
    DISCONNECTED: str = 'disconnected'
    

__all__ = ['com_info_status_type']