package net.wavem.uvc.ros.std_msgs.gateway.chatter.request

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.mqtt.infra.MqttLogger
import org.springframework.stereotype.Component

@Component
class ChatterRequestHandler (
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<net.wavem.uvc.ros.std_msgs.msg.String> {

    override fun handle(data: net.wavem.uvc.ros.std_msgs.msg.String) {
        log.info(REQUEST_CLASS_TYPE, "[$MQTT_CHATTER_REQUEST_TOPIC] message arrived [$data]")
        mqttOutboundGateway.publish(topic = MQTT_CHATTER_REQUEST_TOPIC, data = data)
    }

    companion object {
        const val REQUEST_CLASS_TYPE: String = "REQ"
        const val MQTT_CHATTER_REQUEST_TOPIC: String = "/request/chatter"
    }
}