package net.wavem.uvc.rms.gateway.location.domain.task

data class LocationTaskInfo(
    val jobGroup: String?,
    val jobKind: String?,
    val taskStatus: String?
) {
}