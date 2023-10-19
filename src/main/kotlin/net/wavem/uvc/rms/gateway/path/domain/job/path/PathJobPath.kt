package net.wavem.uvc.rms.gateway.path.domain.job.path

import com.google.gson.JsonArray
import net.wavem.uvc.rms.gateway.path.domain.job.kind.PathJobKindType

data class PathJobPath(
    val areaClsf : String?,
    val locationList : JsonArray?,
    val jobKindType : PathJobKindType?
) {
}