package net.wavem.uvc.rms.gateway.event.domain.com_info

data class ComInfo(
    val status : String?,
    val robotIP : String?,
    val port : String?,
    val mqttIP : String?,
    val mqttPort : String?
) {
}