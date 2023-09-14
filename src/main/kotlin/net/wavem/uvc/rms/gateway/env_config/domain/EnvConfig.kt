package net.wavem.uvc.rms.gateway.env_config.domain

import net.wavem.uvc.rms.common.domain.header.RmsCommonHeader
import net.wavem.uvc.rms.gateway.env_config.domain.set_info.EnvConfigSetInfo

data class EnvConfig(
    val header: RmsCommonHeader,
    val setInfo: EnvConfigSetInfo
) {
}