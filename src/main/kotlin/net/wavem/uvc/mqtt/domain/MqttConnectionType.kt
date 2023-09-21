package net.wavem.uvc.mqtt.domain

enum class MqttConnectionType(
    val type : String
) {
    TO_BRIDGE("TO_ROS"),
    FROM_BRIDGE("FROM_BRIDGE"),
    TO_RMS("TO_RMS"),
    FROM_RMS("FROM_RMS")
}