package net.wavem.uvc.ros.std_msgs.gateway.chatter.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("chatter")
data class ChatterProperties(
    val default : String,
    val toBridge : String,
    val fromBridge : String,
    val qos : Int
) {
}