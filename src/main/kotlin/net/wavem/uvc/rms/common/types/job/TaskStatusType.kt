package net.wavem.uvc.rms.common.types.job

enum class TaskStatusType(
    val type : String
) {
    ASSIGNED("assigned"),
    UNASSIGNED("unassigned")
}