package net.wavem.uvc.rms.gateway.path.domain

import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.path.domain.job.info.PathJobInfo
import net.wavem.uvc.rms.gateway.path.domain.job.kind.PathJobKind
import net.wavem.uvc.rms.gateway.path.domain.job.path.PathJobPath

data class Path(
    val header: Header?,
    val jobInfo: PathJobInfo?,
    val jobPath: PathJobPath?,
    val jobKind: PathJobKind?
) {
}