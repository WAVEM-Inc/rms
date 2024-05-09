import json;
from typing import Any;
from datetime import datetime;
from rosbridge_library.internal import message_conversion;

def ros_message_to_json(message: Any) -> Any:
    try:
        return json.dumps(obj=message_conversion.extract_values(inst=message), indent=4);
    except Exception as e:
        print(f"ros_message_to_json : {e}");
        
def json_to_ros_message(json: Any, type: Any) -> Any:
    try:
        return message_conversion.populate_instance(msg=json, inst=type());
    except message_conversion.NonexistentFieldException as nefe:
        print(f"json_to_ros_message : {nefe}");

def get_current_time() -> str:
    current_time: datetime = datetime.now();
    formatted_time: str = current_time.strftime("%y%m%d%H%M%S%f")[:-3];

    return formatted_time;


__all__: list[str] = ["utils"];