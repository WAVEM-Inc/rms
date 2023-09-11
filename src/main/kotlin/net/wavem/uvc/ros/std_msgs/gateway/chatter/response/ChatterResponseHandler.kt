package net.wavem.uvc.ros.std_msgs.gateway.chatter.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.std_msgs.gateway.chatter.domain.ChatterProperties
import org.springframework.stereotype.Component

@Component
class ChatterResponseHandler(
    private val chatterProperties: ChatterProperties,
    private val mqttService: MqttService<net.wavem.uvc.ros.std_msgs.msg.String>
) {

    var stdString: net.wavem.uvc.ros.std_msgs.msg.String? = null

    fun handle(string: net.wavem.uvc.ros.std_msgs.msg.String) {
        val data: net.wavem.uvc.ros.std_msgs.msg.String = mqttService.handle(
            connectionType = MqttConnectionType.RESP,
            topic = chatterProperties.fromBridge,
            data = string
        )

        stdString = data

        println("stdString returned data : [$data]")
    }
}