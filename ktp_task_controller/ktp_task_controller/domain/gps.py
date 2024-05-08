from sensor_msgs.msg import NavSatFix;

_gps: NavSatFix = None;

@staticmethod
def get_gps() -> NavSatFix:
    return _gps;

@staticmethod
def set_gps(gps: NavSatFix) -> None:
    global _gps;
    _gps = gps;
    

__all__: list[str] = ["gps"];