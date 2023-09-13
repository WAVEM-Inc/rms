package net.wavem.uvc.rms.domain.job.types

enum class TaskStatusType(
    val type: String
) {
    ASSIGNED("assigned"),
    UNASSIGNED("unassigned")
}