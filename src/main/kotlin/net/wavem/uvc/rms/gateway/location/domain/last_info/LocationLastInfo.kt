package net.wavem.uvc.rms.gateway.location.domain.last_info

import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition

data class LocationLastInfo(
    val location : LocationPosition,
    val areaClsf : String,
    val floor : String,
    val batteryLevel : Int,
    val velocity : Double,
    val totalDist : Int
) {
}