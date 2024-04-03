import json;

from rosbridge_library.internal import message_conversion;
from typing import Any;
from datetime import datetime;


def ros_message_dumps(message: Any) -> str:
    return json.dumps(obj=message_conversion.extract_values(inst=message), indent=4);


def get_current_time() -> str:
    current_time: datetime = datetime.now();
    formatted_time: str = current_time.strftime("%y%m%d%H%M%S%f")[:-3];

    return formatted_time;
