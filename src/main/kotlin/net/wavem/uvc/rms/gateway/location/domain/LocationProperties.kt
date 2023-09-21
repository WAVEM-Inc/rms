package net.wavem.uvc.rms.gateway.location.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("location")
class LocationProperties(
    val topic : String,
    val qos : Int,
    val retain : Boolean
) {
}