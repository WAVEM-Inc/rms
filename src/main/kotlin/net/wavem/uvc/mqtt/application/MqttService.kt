package net.wavem.uvc.mqtt.application

import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import org.springframework.stereotype.Service

@Service
class MqttService<T>(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway<T>
){

    fun handle(connectionType: MqttConnectionType, topic: String, data: T) {
        log.info(connectionType.type, "[${topic}] message arrived : [$data]")
        mqttOutboundGateway.publish(topic = topic, data = data)
    }
}