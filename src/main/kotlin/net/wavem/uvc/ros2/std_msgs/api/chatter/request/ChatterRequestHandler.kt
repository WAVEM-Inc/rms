package net.wavem.uvc.ros2.std_msgs.api.chatter.request

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import org.springframework.stereotype.Component

@Component
class ChatterRequestHandler(
    private val logger: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {
    fun handle(stdString: net.wavem.uvc.ros2.std_msgs.msg.String) {
        logger.info(REQUEST_CLASS_TYPE, "[$MQTT_CHATTER_REQUEST_TOPIC] message arrived [$stdString]")
        mqttOutboundGateway.publish(topic = MQTT_CHATTER_REQUEST_TOPIC, data = stdString)
    }

    companion object {
        const val REQUEST_CLASS_TYPE: String = "REQ"
        const val MQTT_CHATTER_REQUEST_TOPIC: String = "/request/chatter"
    }
}