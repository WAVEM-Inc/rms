package net.wavem.uvc.rms.gateway.event.domain

import net.wavem.uvc.rms.domain.header.Header
import net.wavem.uvc.rms.domain.job.JobInfo
import net.wavem.uvc.rms.gateway.event.domain.com_info.ComInfo
import net.wavem.uvc.rms.gateway.event.domain.event_info.EventInfo

data class Event(
    val header: Header?,
    val jobInfo: JobInfo?,
    val eventInfo: EventInfo?,
    val comInfo: ComInfo?
) {
}