package net.wavem.uvc.rms.gateway.event.dto

import com.sun.tools.javac.jvm.ByteCodes.ret
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

    fun getEventId() : String {
        return this.eventId
    }

    fun setEventInd(eventId : String) {
        this.eventId = eventId
    }

    fun getEventCd() : String {
        return this.eventCd
    }

    fun setEventCd(eventCd : String) {
        this.eventCd = eventCd
    }

    fun getEventSubCd() : String {
        return this.eventSubCd
    }

    fun setEventSubCd(eventSubCd : String) {
        this.eventSubCd = eventSubCd
    }

    fun getAreaClsf() : String {
        return this.areaClsf
    }

    fun setAreaClsf(areaClsf : String) {
        this.areaClsf = areaClsf
    }

    fun getFloor() : String {
        return this.getFloor()
    }

    fun setFloor(floor : String) {
        this.floor = floor
    }

    fun getBatteryLevel() : Int {
        return this.batteryLevel
    }

    fun setBatteryLevel(batteryLevel : Int) {
        this.batteryLevel = batteryLevel
    }

    fun getLocation() : LocationPosition? {
        return this.location
    }

    fun setLocation(location : LocationPosition?) {
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