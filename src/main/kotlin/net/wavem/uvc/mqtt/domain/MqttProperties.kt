package net.wavem.uvc.mqtt.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("mqtt")
data class MqttProperties(
    val url: String,
    val port: Int,
    val topic: String,
    val qos: Int
) {
    fun connectionInfo() = "$url:$port"
}