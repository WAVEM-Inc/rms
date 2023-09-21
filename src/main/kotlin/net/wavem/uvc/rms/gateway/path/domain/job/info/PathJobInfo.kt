package net.wavem.uvc.rms.gateway.path.domain.job.info

data class PathJobInfo(
    val jobPlanId : String?,
    val jobGroupId : Int?,
    val jobOrderId : Int?,
    val jobGroup : String?,
    val jobKindType : String?,
) {
}