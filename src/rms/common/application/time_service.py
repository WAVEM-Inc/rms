from datetime import datetime


class TimeService():
    
    
    def __init__(self) -> None:
        pass
    
    
    def get_current_datetime() -> str:
        current_datetime: datetime = datetime.now()
        formatted_datetime: str = current_datetime.strftime("%y%m%d%H%M%S")
        return formatted_datetime