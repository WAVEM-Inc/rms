package net.wavem.uvc.rms.gateway.control.domain

import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.control.domain.cmd.ControlCmd

data class Control(
    val header: Header?,
    val controlCmd: ControlCmd?
) {
}