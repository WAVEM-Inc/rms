package net.wavem.uvc.ros.std_msgs.gateway.chatter.request

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.std_msgs.gateway.chatter.domain.ChatterProperties
import net.wavem.uvc.ros.std_msgs.msg.String
import org.springframework.stereotype.Component

@Component
class ChatterRequestHandler (
    private val chatterProperties: ChatterProperties,
    private val mqttService: MqttService<String>
) {

    fun handle(string: String) {
        mqttService.bridge(
            connectionType = MqttConnectionType.REQ,
            topic = chatterProperties.toBridge,
            data = string
        )
    }
}