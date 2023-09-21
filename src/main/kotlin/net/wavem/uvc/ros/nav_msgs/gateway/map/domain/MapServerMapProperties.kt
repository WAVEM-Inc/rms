package net.wavem.uvc.ros.nav_msgs.gateway.map.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("mapservermap")
data class MapServerMapProperties(
    val default : String,
    val toBridge : String,
    val fromBridge : String,
    val qos : Int
) {
}