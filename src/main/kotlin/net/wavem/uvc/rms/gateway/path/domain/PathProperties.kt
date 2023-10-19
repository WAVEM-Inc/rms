package net.wavem.uvc.rms.gateway.path.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("path")
class PathProperties(
    val topic : String,
    val qos : Int,
    val retain : Boolean
) {
}