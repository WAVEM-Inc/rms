package net.wavem.uvc.ros2.std_msgs.handler.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import org.springframework.stereotype.Component

@Component
class ChatterResponseHandler(
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {
    fun handle(stdString: net.wavem.uvc.ros2.std_msgs.msg.String) {
        println("$MQTT_CHATTER_RESPONSE_FLAG [$MQTT_CHATTER_RESPONSE_TOPIC] message arrived : [$stdString]")
        mqttOutboundGateway.publish(topic = MQTT_CHATTER_RESPONSE_TOPIC, data = stdString)
    }

    companion object {
        const val MQTT_CHATTER_RESPONSE_FLAG: String = "[INFO] mqtt callback result from"
        const val MQTT_CHATTER_RESPONSE_TOPIC: String = "/atc/uvc/response/chatter"
    }
}