package net.wavem.uvc.rms.gateway.env_config.domain

import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.env_config.domain.set_info.EnvConfigSetInfo

data class EnvConfig(
    val header: Header?,
    val setInfo: EnvConfigSetInfo?
) {
}