package net.wavem.uvc.rms.gateway.path.domain.job.path

import net.wavem.uvc.rms.gateway.path.domain.job.kind.PathJobKind

data class PathJobPath(
    val areaClsf : String?,
    val locationList : ArrayList<PathJobPathLocation>?,
    val pathJobKindType : PathJobKind?
) {
}