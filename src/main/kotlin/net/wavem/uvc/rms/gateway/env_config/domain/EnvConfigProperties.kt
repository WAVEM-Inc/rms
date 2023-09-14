package net.wavem.uvc.rms.gateway.env_config.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("config")
data class EnvConfigProperties(
    val topic: String,
    val qos: Int,
    val retain: Boolean
) {
}