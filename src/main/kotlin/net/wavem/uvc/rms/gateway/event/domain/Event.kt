package net.wavem.uvc.rms.gateway.event.domain

import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.event.domain.com_info.ComInfo
import net.wavem.uvc.rms.gateway.event.domain.event_info.EventInfo
import net.wavem.uvc.rms.gateway.event.domain.job.EventJobInfo

data class Event(
    val header : Header?,
    val jobInfo : EventJobInfo?,
    val eventInfo : EventInfo?,
    val comInfo : ComInfo?
) {
}