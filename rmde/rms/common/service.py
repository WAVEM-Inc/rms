import os
import json
import uuid
import socket

from configparser import ConfigParser
from datetime import datetime

from typing import Dict


def empty_dict() -> Dict:
        return {}
    

class TimeService():
    
    def __init__(self) -> None:
        pass
    
    
    def get_current_datetime(self) -> str:
        __current_datetime: datetime = datetime.now()
        __formatted_datetime: str = __current_datetime.strftime("%y%m%d%H%M%S")

        return __formatted_datetime
    

class UUIDService():
    
    def __init__(self) -> None:
        pass
    
    def generate_uuid(self) -> str:
        __uuid_obj : uuid.UUID = uuid.uuid4()
        __uuid_str : str = str(__uuid_obj)
        
        return __uuid_str


class ConfigService():
    
    def __init__(self, file_path: str, file_name: str) -> None:
        self.config_parser: ConfigParser = ConfigParser()
        self.__config_file_path: str = os.path.join(file_path, file_name)
    
    @property
    def config_file_path(self) -> str:
        return self.__config_file_path
    
    def read(self) -> ConfigParser:
        self.config_parser.read(self.__config_file_path)
        
        return self.config_parser



class NetworkService():
    
    def __init__(self) -> None:
        pass
    
    def get_local_ip(self) -> str:
        __s: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        __s.connect(("8.8.8.8", 80))
        __ip_address: str = __s.getsockname()[0]

        return __ip_address


__all__ = ['rms_common_service']