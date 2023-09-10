package net.wavem.uvc.mqtt.infra

import org.slf4j.Logger
import org.slf4j.LoggerFactory
import org.springframework.stereotype.Component

@Component
class MqttLogger {
    private val logger: Logger = LoggerFactory.getLogger(this.javaClass)

    fun info(type: String, message: String) {
        when (type) {
            REQUEST_CLASS_TYPE -> logger.info(MQTT_TO_BRIDGE + message)
            RESPONSE_CLASS_TYPE -> logger.info(MQTT_FROM_BRIDGE + message)
            else -> logger.error("Failed to log MQTT")
        }
    }

    fun warn(type: String, message: String) {
        when (type) {
            REQUEST_CLASS_TYPE -> logger.warn(MQTT_TO_BRIDGE + message)
            RESPONSE_CLASS_TYPE -> logger.warn(MQTT_FROM_BRIDGE + message)
            else -> logger.error("Failed to log MQTT")
        }
    }

    fun error(type: String, message: String) {
        when (type) {
            REQUEST_CLASS_TYPE -> logger.error(MQTT_TO_BRIDGE + message)
            RESPONSE_CLASS_TYPE -> logger.error(MQTT_FROM_BRIDGE + message)
            else -> logger.error("Failed to log MQTT")
        }
    }

    companion object {
        const val REQUEST_CLASS_TYPE: String = "REQ"
        const val RESPONSE_CLASS_TYPE: String = "RESP"

        const val MQTT_TO_BRIDGE: String = "MQTT -> Bridge "
        const val MQTT_FROM_BRIDGE: String = "Bridge -> MQTT "
    }
}