package net.wavem.uvc.ros.nav_msgs.gateway.map.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("mapservermap")
data class MapServerMapProperties(
    val requestToBridgeTopic: String,
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}