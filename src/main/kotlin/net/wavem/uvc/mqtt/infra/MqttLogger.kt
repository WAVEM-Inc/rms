package net.wavem.uvc.mqtt.infra

import net.wavem.uvc.mqtt.domain.MqttConnectionType
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class MqttLogger {
    private val logger: Logger = LoggerFactory.getLogger(this.javaClass)

    fun info(type: MqttConnectionType, message: String) {
        when (type) {
            MqttConnectionType.TO_BRIDGE -> logger.info(MQTT_TO_BRIDGE + message)
            MqttConnectionType.FROM_BRIDGE -> logger.info(MQTT_FROM_BRIDGE + message)
            MqttConnectionType.TO_RMS -> logger.info(MQTT_TO_RMS + message)
            MqttConnectionType.FROM_RMS -> logger.info(MQTT_FROM_RMS + message)
            else -> logger.error("Failed to log MQTT")
        }
    }

    fun warn(type: MqttConnectionType, message: String) {
        when (type) {
            MqttConnectionType.TO_BRIDGE -> logger.warn(MQTT_TO_BRIDGE + message)
            MqttConnectionType.FROM_BRIDGE -> logger.warn(MQTT_FROM_BRIDGE + message)
            MqttConnectionType.TO_RMS -> logger.warn(MQTT_TO_RMS + message)
            MqttConnectionType.FROM_RMS -> logger.warn(MQTT_FROM_RMS + message)
            else -> logger.error("Failed to log MQTT")
        }
    }

    fun error(type: MqttConnectionType, message: String) {
        when (type) {
            MqttConnectionType.TO_BRIDGE -> logger.error(MQTT_TO_BRIDGE + message)
            MqttConnectionType.FROM_BRIDGE -> logger.error(MQTT_FROM_BRIDGE + message)
            MqttConnectionType.TO_RMS -> logger.warn(MQTT_TO_RMS + message)
            MqttConnectionType.FROM_RMS -> logger.warn(MQTT_FROM_RMS + message)
            else -> logger.error("Failed to log MQTT")
        }
    }

    companion object {
        const val MQTT_TO_BRIDGE: String = "MQTT -> Bridge "
        const val MQTT_FROM_BRIDGE: String = "Bridge -> MQTT "
        const val MQTT_TO_RMS: String = "MQTT -> RMS"
        const val MQTT_FROM_RMS: String = "RMS -> MQTT"
    }
}