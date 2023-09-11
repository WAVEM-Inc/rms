package net.wavem.uvc.ros.nav_msgs.gateway.odometry.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.nav_msgs.msg.Odometry
import org.springframework.stereotype.Component

@Component
class OdometryResponseHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<Odometry> {

    override fun handle(data: Odometry) {
        log.info(RESPONSE_CLASS_TYPE, "[$MQTT_ODOM_RESPONSE_TOPIC] message arrived [$data]")
        mqttOutboundGateway.publish(topic = MQTT_ODOM_RESPONSE_TOPIC, data = data)
    }

    companion object {
        const val RESPONSE_CLASS_TYPE: String = "RESP"
        const val MQTT_ODOM_RESPONSE_TOPIC: String = "/atc/uvc/response/odom"
    }
}