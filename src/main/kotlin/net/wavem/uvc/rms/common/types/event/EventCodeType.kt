package net.wavem.uvc.rms.common.types.event

enum class EventCodeType(
    val type : String
) {
    STOP("stop"),
    ISOLATE("isolate"),
    CALIBRATE("calibrate"),
    BROKEN("broken")
}