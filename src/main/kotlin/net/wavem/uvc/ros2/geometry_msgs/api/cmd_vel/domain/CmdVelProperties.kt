package net.wavem.uvc.ros2.geometry_msgs.api.cmd_vel.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("cmdvel")
data class CmdVelProperties(
    val requestToBridgeTopic: String,
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}