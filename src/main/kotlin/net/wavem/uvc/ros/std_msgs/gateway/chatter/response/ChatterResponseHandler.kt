package net.wavem.uvc.ros.std_msgs.gateway.chatter.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.rms.domain.RmsCommonProperties
import net.wavem.uvc.ros.std_msgs.gateway.chatter.domain.ChatterProperties
import net.wavem.uvc.ros.std_msgs.msg.String
import org.springframework.stereotype.Component

@Component
class ChatterResponseHandler(
    private val rmsCommonProperties: RmsCommonProperties,
    private val chatterProperties: ChatterProperties,
    private val mqttService: MqttService<String>
) {

    fun handle(string: String) {
        mqttService.handle(
            connectionType = MqttConnectionType.RESP,
            topic = rmsCommonProperties.fromRosTopicFormat + chatterProperties.topic,
            data = string
        )
    }
}