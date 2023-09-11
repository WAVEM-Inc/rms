package net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.geometry_msgs.msg.Twist
import org.springframework.stereotype.Component

@Component
class CmdVelResponseHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<Twist> {

    override fun handle(data: Twist) {
        log.info(RESPONSE_CLASS_TYPE, "[${MQTT_CMD_VEL_RESPONSE_TOPIC}] message arrived [$data]")
        mqttOutboundGateway.publish(topic = MQTT_CMD_VEL_RESPONSE_TOPIC, data = data)
    }

    companion object {
        const val RESPONSE_CLASS_TYPE: String = "RESP"
        const val MQTT_CMD_VEL_RESPONSE_TOPIC: String = "/atc/uvc/response/cmd_vel"
    }
}