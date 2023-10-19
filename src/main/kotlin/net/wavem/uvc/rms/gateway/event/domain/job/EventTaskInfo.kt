package net.wavem.uvc.rms.gateway.event.domain.job

import net.wavem.uvc.rms.common.domain.job.result.JobResult

data class EventTaskInfo(
    val jobPlanId : String?,
    val jobGroupId : String?,
    val jobOrderId : String?,
    val jobGroup : String?,
    val jobKindType : String?,
    val jobResult : JobResult?
) {
}