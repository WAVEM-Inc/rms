package net.wavem.uvc.rms.gateway.location.dto

import net.wavem.uvc.rms.gateway.location.domain.position.LocationPosition

class LocationPositionDTO() {
    private var xpos : Double = 0.0
    private var ypos : Double = 0.0
    private var heading : Double = 0.0

    fun getXpos() : Double {
        return this.xpos
    }

    fun setXpos(xpos : Double) {
        this.xpos = xpos
    }

    fun getYpos() : Double {
        return this.ypos
    }

    fun setYpos(ypos : Double) {
        this.ypos = ypos
    }

    fun getHeading() : Double {
        return this.heading
    }

    fun setHeading(heading : Double) {
        this.heading = heading
    }

    fun build() : LocationPosition {
        return LocationPosition(
            xpos = this.xpos,
            ypos = this.ypos,
            heading = this.heading
        )
    }
}