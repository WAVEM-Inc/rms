package net.wavem.uvc.rms.gateway.event.domain.types

enum class EventCodeType(
    val type: String
) {
    STOP("stop"),
    ISOLATE("isolate"),
    CALIBRATE("calibrate"),
    BROKEN("broken")
}