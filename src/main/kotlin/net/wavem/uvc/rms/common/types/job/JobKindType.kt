package net.wavem.uvc.rms.common.types.job

enum class JobKindType(
    val type : String
) {
    LOAD("load"),
    UNLOAD("unload"),
    CHARGE("charge"),
    WAIT("wait"),
    MOVE("move")
}