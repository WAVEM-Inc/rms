package net.wavem.uvc.rms.gateway.config.domain

import net.wavem.uvc.rms.domain.RmsHeader

data class EnvConfig(
    val header: RmsHeader,
    val setInfo: EnvInfo
) {
}