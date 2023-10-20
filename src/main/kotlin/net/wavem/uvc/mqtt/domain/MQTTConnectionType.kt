package net.wavem.uvc.mqtt.domain

enum class MQTTConnectionType(
    val type : String
) {
    TO_UVC("TO_UVC"),
    FROM_UVC("FROM_UVC"),
    TO_RMS("TO_RMS"),
    FROM_RMS("FROM_RMS")
}