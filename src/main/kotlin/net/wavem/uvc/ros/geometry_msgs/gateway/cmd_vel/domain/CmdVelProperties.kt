package net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("cmdvel")
data class CmdVelProperties(
    val default : String,
    val toBridge : String,
    val fromBridge : String,
    val qos : Int
) {
}