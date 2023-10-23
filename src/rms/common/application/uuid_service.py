import uuid

class UUIDService():
    
    def __init__(self) -> None:
        pass
    
    def generate_uuid(self) -> str:
        uuid_obj : uuid.UUID = uuid.uuid4()
        uuid_str : str = str(uuid_obj)
        return uuid_str