package net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.geometry_msgs.gateway.cmd_vel.domain.CmdVelProperties
import net.wavem.uvc.ros.geometry_msgs.msg.twist.Twist
import org.springframework.stereotype.Component

@Component
class CmdVelResponseHandler(
    private val cmdVelProperties : CmdVelProperties,
    private val mqttService : MqttService<Twist>
) {

    fun handle(twist : Twist) {
        mqttService.bridge(
            connectionType = MqttConnectionType.FROM_BRIDGE,
            topic = cmdVelProperties.fromBridge,
            data = twist
        )
    }
}