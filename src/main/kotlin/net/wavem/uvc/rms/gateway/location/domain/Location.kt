package net.wavem.uvc.rms.gateway.location.domain

import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.location.domain.job.LocationJobInfo
import net.wavem.uvc.rms.gateway.location.domain.last_info.LocationLastInfo

data class Location(
    val header : Header?,
    val jobInfo : LocationJobInfo?,
    val lastInfo : LocationLastInfo?
) {
}