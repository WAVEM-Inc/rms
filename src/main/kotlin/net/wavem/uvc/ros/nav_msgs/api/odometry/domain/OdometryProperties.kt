package net.wavem.uvc.ros.nav_msgs.api.odometry.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("odometry")
data class OdometryProperties(
    val requestToBridgeTopic: String,
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}