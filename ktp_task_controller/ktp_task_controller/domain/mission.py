from ktp_data_msgs.msg import Mission;

_mission: Mission = None;

@staticmethod
def get_mission() -> Mission:
    return _mission;

@staticmethod
def set_mission(mission: Mission | None) -> None:
    global _mission;
    _mission = mission;
    

__all__: list[str] = ["mission"];