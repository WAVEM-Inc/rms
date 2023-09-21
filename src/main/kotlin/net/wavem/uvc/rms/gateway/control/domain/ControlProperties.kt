package net.wavem.uvc.rms.gateway.control.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("control")
class ControlProperties(
    val topic : String,
    val qos : Int,
    val retain : Boolean
) {
}