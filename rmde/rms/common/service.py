import os
import json
import uuid
import socket

from configparser import ConfigParser
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


class ConfigService():
    
    def __init__(self, file_path: str, file_name: str) -> None:
        self.__config_parser__: ConfigParser = ConfigParser()
        self.__config_file_path__: str = os.path.join(file_path, file_name)
    
    
    def read(self) -> ConfigParser:
        self.__config_parser__.read(self.__config_file_path__)
        
        return self.__config_parser__
    

class NetworkService():
    
    def __init__(self) -> None:
        pass
    
    def get_local_ip(self) -> str:
        s: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_address: str = s.getsockname()[0]
        return ip_address


__all__ = ['common_service']