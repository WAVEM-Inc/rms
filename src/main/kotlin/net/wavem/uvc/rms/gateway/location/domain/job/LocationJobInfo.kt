package net.wavem.uvc.rms.gateway.location.domain.job

import net.wavem.uvc.rms.gateway.location.domain.task.LocationTaskInfo

data class LocationJobInfo(
    val jobPlanId: String?,
    val jobGroupId: Int?,
    val jobOrderId: Int?,
    val taskInfo: LocationTaskInfo?
) {
}