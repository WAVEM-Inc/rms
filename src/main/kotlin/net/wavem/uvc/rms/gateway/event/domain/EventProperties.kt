package net.wavem.uvc.rms.gateway.event.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("event")
data class EventProperties(
    val topic: String,
    val qos: Int,
    val retain: Boolean
) {
}