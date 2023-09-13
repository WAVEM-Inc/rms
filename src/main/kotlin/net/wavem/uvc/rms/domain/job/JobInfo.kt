package net.wavem.uvc.rms.domain.job

import net.wavem.uvc.rms.domain.job.result.JobResult
import net.wavem.uvc.rms.domain.job.task.TaskInfo

data class JobInfo(
    val jobPlanId: String,
    val jobGroupId: Int,
    val jobOrderId: Int,
    val taskInfo: TaskInfo?,
    val jobResult: JobResult?
) {
}