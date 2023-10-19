package net.wavem.uvc.rms.common.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("rms")
data class RmsCommonProperties(
    val toROS : String,
    val fromROS : String
) {

}