package net.wavem.uvc.rms.gateway.control.request

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.gateway.control.domain.Control
import net.wavem.uvc.rms.gateway.control.domain.ControlProperties
import org.springframework.stereotype.Component

@Component
class ControlRequestHandler(
    val log: MqttLogger,
    val controlProperties: ControlProperties,
    val mqttService: MqttService<String>
) {

    fun handle(control: Control) {
        val controlJson: JsonObject = Gson().toJsonTree(control).asJsonObject
        log.info(MqttConnectionType.FROM_RMS, "controlJson : [$controlJson]")
        mqttService.bridge(
            MqttConnectionType.TO_BRIDGE,
            "/test/control",
            controlJson.toString()
        )
    }
}