package net.wavem.uvc.ros.geometry_msgs.api.cmd_vel.request

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.geometry_msgs.msg.Twist
import org.springframework.stereotype.Component

@Component
class CmdVelRequestHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {

    fun handle(twist: Twist) {
        log.info(REQUEST_CLASS_TYPE, "[${MQTT_CMD_VEL_REQUEST_TOPIC}] message arrived [$twist]")
        mqttOutboundGateway.publish(topic = MQTT_CMD_VEL_REQUEST_TOPIC, data = twist)
    }

    companion object {
        const val REQUEST_CLASS_TYPE: String = "REQ"
        const val MQTT_CMD_VEL_REQUEST_TOPIC: String = "/request/cmd_vel"
    }
}