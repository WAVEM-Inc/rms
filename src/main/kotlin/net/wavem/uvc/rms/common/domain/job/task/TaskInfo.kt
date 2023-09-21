package net.wavem.uvc.rms.common.domain.job.task

data class TaskInfo(
    val jobGroup : String?,
    val jobKind : String?,
    val taskStatus : String?
) {
}