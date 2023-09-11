package net.wavem.uvc.ros.std_msgs.api.chatter.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import org.springframework.stereotype.Component

@Component
class ChatterResponseHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {
    fun handle(stdString: net.wavem.uvc.ros.std_msgs.msg.String) {
        log.info(RESPONSE_CLASS_TYPE, "[$MQTT_CHATTER_RESPONSE_TOPIC] message arrived : [$stdString]")
        mqttOutboundGateway.publish(topic = MQTT_CHATTER_RESPONSE_TOPIC, data = stdString)
    }

    companion object {
        const val RESPONSE_CLASS_TYPE: String = "RESP"
        const val MQTT_CHATTER_RESPONSE_TOPIC: String = "/atc/uvc/response/chatter"
    }
}