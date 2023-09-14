package net.wavem.uvc.rms.common.types.job

enum class JobGroupType(
    val type: String
) {
    SUPPLY("supply"),
    RECOVERY("recovery"),
    MOVE("move"),
    CHARGE("charge"),
    WAIT("wait")
}