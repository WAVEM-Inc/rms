package net.wavem.uvc.rms.domain.job.types

enum class JobGroupType(
    val type: String
) {
    SUPPLY("supply"),
    RECOVERY("recovery"),
    MOVE("move"),
    CHARGE("charge"),
    WAIT("wait")
}