package net.wavem.uvc.rms.gateway.config

import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.ros.std_msgs.msg.String
import org.springframework.stereotype.Component

@Component
class ConfigHandler : MqttHandler<String> {
    override fun handle(data: String) {
        TODO("Not yet implemented")
    }
}