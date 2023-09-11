package net.wavem.uvc.rms.gateway.config.domain

data class EnvInfo(
    val robotType: String,
    val mqttIP: String,
    val mqttPort: String,
    val robotCorpId: String,
    val workCorpId: String,
    val workSiteId: String
) {
}