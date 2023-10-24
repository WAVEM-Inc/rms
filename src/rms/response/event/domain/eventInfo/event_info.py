from rms.response.event.domain.eventInfo.location.event_info_location import EventInfoLocation


class EventInfo():
    def __init__(
        self, 
        eventId: str = None,
        eventCd: str = None,
        eventSubCd: str = None,
        areaClsf: str = None,
        floor: str = None,
        batteryLevel: int = None,
        location: dict = None
    ) -> None:
        self.eventId = eventId
        self.eventCd = eventCd
        self.eventSubCd = eventSubCd
        self.areaClsf = areaClsf
        self.floor = floor
        self.batteryLevel = batteryLevel
        self.location = location