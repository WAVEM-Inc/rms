package net.wavem.uvc.rms.gateway.path.domain.job.info

data class PathJobInfo(
    val jobPlanId : String?,
    val jobGroupId : String?,
    val jobOrderId : String?,
    val jobGroup : String?,
    val jobKindType : String?,
) {
}