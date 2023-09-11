package net.wavem.uvc.mqtt.domain

enum class MqttConnectionType(
    val type: String
) {
    REQ("REQ"),
    RESP("RESP")
}