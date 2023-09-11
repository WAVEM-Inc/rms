package net.wavem.uvc.ros.std_msgs.gateway.chatter.response

import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.std_msgs.gateway.chatter.domain.ChatterProperties
import org.springframework.stereotype.Component

@Component
class ChatterResponseHandler(
    private val log: MqttLogger,
    private val chatterProperties: ChatterProperties,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<net.wavem.uvc.ros.std_msgs.msg.String> {
    override fun handle(data: net.wavem.uvc.ros.std_msgs.msg.String) {
        log.info(MqttConnectionType.RESP.type, "[${chatterProperties.responseFromBridgeTopic}] message arrived : [$data]")
        mqttOutboundGateway.publish(topic = chatterProperties.responseFromBridgeTopic, data = data)
    }
}