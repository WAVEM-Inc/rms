package net.wavem.uvc.rms.gateway.location.domain

import net.wavem.uvc.rms.domain.header.Header
import net.wavem.uvc.rms.domain.job.JobInfo
import net.wavem.uvc.rms.gateway.location.domain.last_info.LastInfo

data class Location(
    val header: Header,
    val jobInfo: JobInfo,
    val lastInfo: LastInfo
) {
}