package net.wavem.uvc.ros.nav_msgs.gateway.odometry.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("odometry")
data class OdometryProperties(
    val default : String,
    val toBridge : String,
    val fromBridge : String,
    val qos : Int
) {
}