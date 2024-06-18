import os;
import json;
from rosbridge_library.internal import message_conversion;
from rclpy.impl.rcutils_logger import RcutilsLogger;
from datetime import datetime;
from pyproj import Proj;
from math import sqrt;
from typing import Any;


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
    
def get_initial_node_id(log: RcutilsLogger) -> str | None:
    try:
        home_directory: str = os.path.expanduser("~");
        file_path: str = f"{home_directory}/RobotData/route/route.json";
            
        with open(file_path, "r", encoding="utf-8") as f:
            file_content: Any = json.load(f);
            log.info(f"Get initial node id : {json.dumps(obj=file_content, indent=4)}");
            
            initial_node_id: Any | str = file_content["initial_node"];
            log.info(f"========= initial node id : {initial_node_id} =========");
            
            if initial_node_id is None:
                raise Exception();
            return initial_node_id;
    except Exception as e:
        print(f"Get initial node id : {e}");
        return None;
        

__all__: list[str] = ["utils"];