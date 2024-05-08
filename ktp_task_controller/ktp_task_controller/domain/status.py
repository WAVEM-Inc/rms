
_driving_status: int = 0;

@staticmethod
def get_driving_status() -> int:
    return _driving_status;

@staticmethod
def set_driving_status(driving_status: int) -> None:
    global _driving_status;
    _driving_status = driving_status;
    

__all__: list[str] = ["status"];