package net.wavem.uvc.mqtt.infra

import net.wavem.uvc.mqtt.domain.MqttConnectionType
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class MqttLogger {
    private val logger: Logger = LoggerFactory.getLogger(this.javaClass)

    fun info(type: MqttConnectionType, message: String) {
        val loggingMessage: String = "$MQTT_TO_BRIDGE $message"

        when (type) {
            MqttConnectionType.TO_BRIDGE -> logger.info(loggingMessage)
            MqttConnectionType.FROM_BRIDGE -> logger.info(loggingMessage)
            MqttConnectionType.TO_RMS -> logger.info(loggingMessage)
            MqttConnectionType.FROM_RMS -> logger.info(loggingMessage)
            else -> logger.error("Failed to log MQTT")
        }
    }

    fun warn(type: MqttConnectionType, message: String) {
        val loggingMessage: String = "$MQTT_TO_BRIDGE $message"

        when (type) {
            MqttConnectionType.TO_BRIDGE -> logger.warn(loggingMessage)
            MqttConnectionType.FROM_BRIDGE -> logger.warn(loggingMessage)
            MqttConnectionType.TO_RMS -> logger.warn(loggingMessage)
            MqttConnectionType.FROM_RMS -> logger.warn(loggingMessage)
            else -> logger.error("Failed to log MQTT")
        }
    }

    fun error(type: MqttConnectionType, message: String) {
        val loggingMessage: String = "$MQTT_TO_BRIDGE $message"

        when (type) {
            MqttConnectionType.TO_BRIDGE -> logger.error(loggingMessage)
            MqttConnectionType.FROM_BRIDGE -> logger.error(loggingMessage)
            MqttConnectionType.TO_RMS -> logger.warn(loggingMessage)
            MqttConnectionType.FROM_RMS -> logger.warn(loggingMessage)
            else -> logger.error("Failed to log MQTT")
        }
    }

    companion object {
        const val MQTT_TO_BRIDGE: String = "[MQTT -> Bridge]"
        const val MQTT_FROM_BRIDGE: String = "[Bridge -> MQTT]"
        const val MQTT_TO_RMS: String = "[MQTT -> RMS]"
        const val MQTT_FROM_RMS: String = "[RMS -> MQTT]"
    }
}