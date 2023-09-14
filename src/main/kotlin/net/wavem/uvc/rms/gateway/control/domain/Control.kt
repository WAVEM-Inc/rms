package net.wavem.uvc.rms.gateway.control.domain

import net.wavem.uvc.rms.common.domain.header.RmsCommonHeader
import net.wavem.uvc.rms.gateway.control.domain.cmd.ControlCmd

data class Control(
    val header: RmsCommonHeader?,
    val controlCmd: ControlCmd?
) {
}