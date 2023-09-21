package net.wavem.uvc.rms.gateway.event.domain.event_info

import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition

data class EventInfo(
    val eventId : String?,
    val eventCd : String?,
    val eventSubCd : String?,
    val areaClsf : String?,
    val floor : String?,
    val batteryLevel : Int?,
    val location : LocationPosition?
) {
}