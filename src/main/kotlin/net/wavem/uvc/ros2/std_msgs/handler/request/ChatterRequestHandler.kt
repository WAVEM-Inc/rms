package net.wavem.uvc.ros2.std_msgs.handler.request

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import org.springframework.stereotype.Component

@Component
class ChatterRequestHandler(
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {
    fun handle(stdString: net.wavem.uvc.ros2.std_msgs.msg.String) {
        println("$MQTT_CHATTER_REQUEST_FLAG [$MQTT_CHATTER_REQUEST_TOPIC] message arrived [$stdString]")
        mqttOutboundGateway.publish(topic = MQTT_CHATTER_REQUEST_TOPIC, data = stdString)
    }

    companion object {
        const val MQTT_CHATTER_REQUEST_FLAG: String = "[INFO] mqtt callback from"
        const val MQTT_CHATTER_REQUEST_TOPIC: String = "/request/chatter"
    }
}