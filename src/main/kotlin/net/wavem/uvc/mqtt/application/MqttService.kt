package net.wavem.uvc.mqtt.application

import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import org.springframework.stereotype.Service

@Service
class MqttService<T>(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway<T>
) {

    fun processROSData(connectionType: MqttConnectionType, topic: String, data: T) : T {
        log.info(connectionType, "[$topic] message arrived : [$data]")

        return data
    }

    fun bridge(connectionType: MqttConnectionType, topic: String, data: T) {
        log.info(connectionType, "bridge message [$data] to [$topic]")
        mqttOutboundGateway.publish(topic = topic, data = data)
    }
}