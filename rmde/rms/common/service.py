import json
import uuid
from datetime import datetime


class JsonEncoder(json.JSONEncoder):
    def default(self, obj):
        if hasattr(obj, '__dict__'):
            return {'__class__': obj.__class__.__name__, '__dict__': obj.__dict__}
        return super().default(obj)
    

class TimeService():
    def __init__(self) -> None:
        pass
    
    
    def get_current_datetime(self) -> str:
        current_datetime: datetime = datetime.now()
        formatted_datetime: str = current_datetime.strftime("%y%m%d%H%M%S")
        return formatted_datetime
    

class UUIDService():
    
    def __init__(self) -> None:
        pass
    
    def generate_uuid(self) -> str:
        uuid_obj : uuid.UUID = uuid.uuid4()
        uuid_str : str = str(uuid_obj)
        return uuid_str
    

__all__ = ['common_service']