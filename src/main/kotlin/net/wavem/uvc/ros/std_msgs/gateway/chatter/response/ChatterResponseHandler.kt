package net.wavem.uvc.ros.std_msgs.gateway.chatter.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.std_msgs.gateway.chatter.domain.ChatterProperties
import net.wavem.uvc.ros.std_msgs.msg.string.String
import org.springframework.stereotype.Component

@Component
class ChatterResponseHandler(
    private val chatterProperties : ChatterProperties,
    private val mqttService : MqttService<String>
) {

    var stdString : String? = null

    fun handle(string : String) {
        val data : String = mqttService.processROSData(
            connectionType = MqttConnectionType.FROM_BRIDGE,
            topic = chatterProperties.fromBridge,
            data = string
        )

        stdString = data
    }
}