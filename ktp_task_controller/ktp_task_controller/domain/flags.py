to_source_flag: bool = False;
to_dest_flag: bool = False;
returning_flag: bool = False;
driving_flag: bool = False;

@staticmethod
def get_to_source_flag() -> bool:
    return to_source_flag;

@staticmethod
def set_to_source_flag(flag: bool) -> None:
    global to_source_flag;
    to_source_flag = flag;

@staticmethod
def get_to_dest_flag() -> bool:
    return to_dest_flag;

@staticmethod
def set_to_dest_flag(flag: bool) -> None:
    global to_dest_flag;
    to_dest_flag = flag;

@staticmethod
def get_returning_flag() -> bool:
    return returning_flag;

@staticmethod
def set_returning_flag(flag: bool) -> None:
    global returning_flag;
    returning_flag = flag;

@staticmethod
def get_driving_flag() -> bool:
    return driving_flag;

@staticmethod
def set_driving_flag(flag: bool) -> None:
    global driving_flag;
    driving_flag = flag;
    
    
__all__: list[str] = ["flag"];