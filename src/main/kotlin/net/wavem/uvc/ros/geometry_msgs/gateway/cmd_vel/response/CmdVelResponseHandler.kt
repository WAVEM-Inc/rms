package net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.geometry_msgs.msg.Twist
import org.springframework.stereotype.Component

@Component
class CmdVelResponseHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {

    fun handle(twist: Twist) {
        log.info(RESPONSE_CLASS_TYPE, "[${MQTT_CMD_VEL_RESPONSE_TOPIC}] message arrived [$twist]")
        mqttOutboundGateway.publish(topic = MQTT_CMD_VEL_RESPONSE_TOPIC, data = twist)
    }

    companion object {
        const val RESPONSE_CLASS_TYPE: String = "RESP"
        const val MQTT_CMD_VEL_RESPONSE_TOPIC: String = "/atc/uvc/response/cmd_vel"
    }
}