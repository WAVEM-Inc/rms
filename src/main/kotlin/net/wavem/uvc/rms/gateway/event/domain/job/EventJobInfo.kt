package net.wavem.uvc.rms.gateway.event.domain.job

import net.wavem.uvc.rms.common.domain.job.result.RmsCommonJobResult

data class EventJobInfo(
    val jobPlanId: String?,
    val jobGroupId: Int?,
    val jobOrderId: Int?,
    val jobGroup: String?,
    val jobKindType: String?,
    val jobResult: RmsCommonJobResult?
) {
}