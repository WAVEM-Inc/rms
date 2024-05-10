import json;
from rosbridge_library.internal import message_conversion;
from datetime import datetime;
from pyproj import Proj;
from math import radians, sin, cos, sqrt, atan2;
from typing import Any;
from typing import Tuple;


def ros_message_dumps(message: Any) -> str:
    return json.dumps(obj=message_conversion.extract_values(inst=message), indent=4);

def get_current_time() -> str:
    current_time: datetime = datetime.now();
    formatted_time: str = current_time.strftime("%y%m%d%H%M%S%f")[:-3];

    return formatted_time;

def convert_latlon_to_utm(latitude: float, longitude: float) -> Any:
    utm_converter: Proj = Proj(proj="utm", zone=52, ellps="WGS84");

    utm_x, utm_y = utm_converter(longitude, latitude);

    return utm_x, utm_y;

def distance_between(x1: float, y1: float, x2: float, y2: float) -> float:
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);


__all__: list[str] = ["utils"];