package net.wavem.uvc.ros2.std_msgs.api.chatter.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("chatter")
data class ChatterProperties(
    val requestToBridgeTopic: String,
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}