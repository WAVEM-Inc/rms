package net.wavem.uvc.rms.gateway.control.domain.cmd

data class ControlCmd(
    val ready: Boolean,
    val move: Boolean,
    val stop: Boolean
) {
}