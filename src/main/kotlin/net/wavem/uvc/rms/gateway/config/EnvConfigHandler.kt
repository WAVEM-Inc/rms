package net.wavem.uvc.rms.gateway.config

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.rms.gateway.config.domain.EnvConfig
import org.springframework.stereotype.Component

@Component
class EnvConfigHandler(
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<EnvConfig> {
    override fun handle(data: EnvConfig) {

    }

    companion object {
        const val REQUEST_CLASS_TYPE: String = "REQ"
    }
}