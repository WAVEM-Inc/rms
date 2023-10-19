package net.wavem.uvc.mqtt.infra

import net.wavem.uvc.mqtt.domain.MQTTConnectionType
import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class MQTTLogger {
    private val logger : Logger = LoggerFactory.getLogger(this.javaClass)

    fun info(type : MQTTConnectionType, message : String) {
        val loggingMessage : String = "$MQTT_TO_BRIDGE $message"

        when (type) {
            MQTTConnectionType.TO_BRIDGE -> logger.info(loggingMessage)
            MQTTConnectionType.FROM_BRIDGE -> logger.info(loggingMessage)
            MQTTConnectionType.TO_RMS -> logger.info(loggingMessage)
            MQTTConnectionType.FROM_RMS -> logger.info(loggingMessage)
        }
    }

    fun warn(type : MQTTConnectionType, message : String) {
        val loggingMessage : String = "$MQTT_TO_BRIDGE $message"

        when (type) {
            MQTTConnectionType.TO_BRIDGE -> logger.warn(loggingMessage)
            MQTTConnectionType.FROM_BRIDGE -> logger.warn(loggingMessage)
            MQTTConnectionType.TO_RMS -> logger.warn(loggingMessage)
            MQTTConnectionType.FROM_RMS -> logger.warn(loggingMessage)
        }
    }

    fun error(type : MQTTConnectionType, message : String) {
        val loggingMessage : String = "$MQTT_TO_BRIDGE $message"

        when (type) {
            MQTTConnectionType.TO_BRIDGE -> logger.error(loggingMessage)
            MQTTConnectionType.FROM_BRIDGE -> logger.error(loggingMessage)
            MQTTConnectionType.TO_RMS -> logger.error(loggingMessage)
            MQTTConnectionType.FROM_RMS -> logger.error(loggingMessage)
        }
    }

    private companion object {
        const val MQTT_TO_BRIDGE : String = "[MQTT -> Bridge]"
        const val MQTT_FROM_BRIDGE : String = "[Bridge -> MQTT]"
        const val MQTT_TO_RMS : String = "[MQTT -> RMS]"
        const val MQTT_FROM_RMS : String = "[RMS -> MQTT]"
    }
}