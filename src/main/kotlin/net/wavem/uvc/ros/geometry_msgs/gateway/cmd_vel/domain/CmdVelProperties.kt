package net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("cmdvel")
data class CmdVelProperties(
    val topic: String,
    val requestToBridgeTopic: String,
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}