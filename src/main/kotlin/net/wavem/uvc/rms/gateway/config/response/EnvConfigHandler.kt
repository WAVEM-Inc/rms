package net.wavem.uvc.rms.gateway.config.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.rms.gateway.config.domain.EnvConfig
import net.wavem.uvc.ros.std_msgs.gateway.chatter.response.ChatterResponseHandler
import org.springframework.stereotype.Component

@Component
class EnvConfigHandler(
    private val chatterResponseHandler: ChatterResponseHandler,
    private val mqttService: MqttService<EnvConfig>
) {

    fun handle(envConfig: EnvConfig) {
        val data: net.wavem.uvc.ros.std_msgs.msg.String? = chatterResponseHandler.stdString

        if (data != null) {
            println("===== envConfig stdString test result : [$data] =====")
        } else {
            println("envConfig stdString test result is null")
        }
    }
}