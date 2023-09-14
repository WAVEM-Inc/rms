package net.wavem.uvc.rms.gateway.location.domain

import net.wavem.uvc.rms.common.domain.header.RmsCommonHeader
import net.wavem.uvc.rms.gateway.location.domain.job.LocationJobInfo
import net.wavem.uvc.rms.gateway.location.domain.last_info.LocationLastInfo

data class Location(
    val header: RmsCommonHeader?,
    val jobInfo: LocationJobInfo?,
    val locationLastInfo: LocationLastInfo?
) {
}