package net.wavem.uvc.rms.domain.job.result

data class JobResult(
    val status: String,
    val startTime: String,
    val endTime: String,
    val startBatteryLevel: Int,
    val endBatteryLevel: Int,
    val dist: Int
) {
}