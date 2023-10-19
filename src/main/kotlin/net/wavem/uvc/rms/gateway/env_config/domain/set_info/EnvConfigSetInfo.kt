package net.wavem.uvc.rms.gateway.env_config.domain.set_info

data class EnvConfigSetInfo(
    val robotType : String?,
    val mqttIP : String?,
    val mqttPort : String?,
    val robotCorpId : String?,
    val workCorpId : String?,
    val workSiteId : String?,
    val batteryEvent : String?
) {
}