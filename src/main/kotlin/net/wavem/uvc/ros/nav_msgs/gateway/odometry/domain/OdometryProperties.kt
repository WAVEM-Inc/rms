package net.wavem.uvc.ros.nav_msgs.gateway.odometry.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("odometry")
data class OdometryProperties(
    val topic: String,
    val requestToBridgeTopic: String,
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}