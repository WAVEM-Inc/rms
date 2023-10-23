package net.wavem.uvc.rms.gateway.event.dto

import net.wavem.uvc.rms.gateway.event.domain.event_info.EventInfo
import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition

class EventInfoDTO() {
    var eventId : String = ""
    var eventCd : String = ""
    var eventSubCd : String = ""
    var areaClsf : String = ""
    var floor : String = ""
    var batteryLevel : Int = 0
    var location : LocationPosition? = null

    constructor(
        eventId : String,
        eventCd : String,
        eventSubCd : String,
        areaClsf : String,
        floor : String,
        batteryLevel : Int,
        location : LocationPosition?
    ) : this() {
        this.eventId = eventId
        this.eventCd = eventCd
        this.eventSubCd = eventSubCd
        this.areaClsf = areaClsf
        this.floor = floor
        this.batteryLevel = batteryLevel
        this.location = location
    }

    fun build() : EventInfo {
        return EventInfo(
            eventId = this.eventId,
            eventCd = this.eventCd,
            eventSubCd = this.eventSubCd,
            areaClsf = this.areaClsf,
            floor = this.floor,
            batteryLevel = this.batteryLevel,
            location = this.location
        )
    }
}