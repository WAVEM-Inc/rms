package net.wavem.uvc.rms.gateway.location.domain.job

import net.wavem.uvc.rms.gateway.location.domain.task.LocationTaskInfo

data class LocationJobInfo(
    val jobPlanId  : String?,
    val jobGroupId : String?,
    val jobOrderId : String?,
    val taskInfo : LocationTaskInfo?
) {
}