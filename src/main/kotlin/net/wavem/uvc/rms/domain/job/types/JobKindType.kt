package net.wavem.uvc.rms.domain.job.types

enum class JobKindType(
    val type: String
) {
    LOAD("load"),
    UNLOAD("unload"),
    CHARGE("charge"),
    WAIT("wait"),
    MOVE("move"),
    PATROL("patrol")
}