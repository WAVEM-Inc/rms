package net.wavem.uvc.rms.gateway.location.dto

import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition

class LocationPositionDTO() {
    var xpos : Double = 0.0
    var ypos : Double = 0.0
    var heading : Double = 0.0

    fun build() : LocationPosition {
        return LocationPosition(
            xpos = this.xpos,
            ypos = this.ypos,
            heading = this.heading
        )
    }
}