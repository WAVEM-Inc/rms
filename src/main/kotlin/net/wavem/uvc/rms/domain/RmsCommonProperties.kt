package net.wavem.uvc.rms.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("rms")
data class RmsCommonProperties(
    val toROS: String,
    val fromROS: String
) {

}