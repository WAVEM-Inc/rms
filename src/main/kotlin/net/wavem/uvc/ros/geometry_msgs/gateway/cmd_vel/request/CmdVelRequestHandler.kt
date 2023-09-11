package net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.request

import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.domain.CmdVelProperties
import net.wavem.uvc.ros.geometry_msgs.msg.Twist
import org.springframework.stereotype.Component

@Component
class CmdVelRequestHandler(
    private val log: MqttLogger,
    private val cmdVelProperties: CmdVelProperties,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<Twist> {

    override fun handle(data: Twist) {
        log.info(MqttConnectionType.REQ.type, "[${cmdVelProperties.requestToBridgeTopic}] message arrived [$data]")
        mqttOutboundGateway.publish(topic = cmdVelProperties.requestToBridgeTopic, data = data)
    }
}