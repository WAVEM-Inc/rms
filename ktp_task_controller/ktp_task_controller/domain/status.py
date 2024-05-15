_driving_status: int = 0;
_mission_total_distance: int = 0;
_last_arrived_node_id: str = "";

@staticmethod
def get_driving_status() -> int:
    return _driving_status;

@staticmethod
def set_driving_status(driving_status: int) -> None:
    global _driving_status;
    _driving_status = driving_status;

@staticmethod
def get_mission_total_distance() -> int:
    return _mission_total_distance;

@staticmethod
def set_mission_total_distance(mission_total_distance: int) -> None:
    global _mission_total_distance;
    _mission_total_distance = mission_total_distance;

@staticmethod
def get_last_arrived_node_id() -> str:
    return _last_arrived_node_id;

@staticmethod
def set_last_arrived_node_id(last_arrived_node_id: str) -> None:
    global _last_arrived_node_id;
    _last_arrived_node_id = last_arrived_node_id;


__all__: list[str] = ["status"];