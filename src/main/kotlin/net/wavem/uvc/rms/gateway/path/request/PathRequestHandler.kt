package net.wavem.uvc.rms.gateway.path.request

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.gateway.path.domain.Path
import net.wavem.uvc.rms.gateway.path.domain.PathProperties
import org.springframework.stereotype.Component

@Component
class PathRequestHandler(
    val log: MqttLogger,
    val pathProperties: PathProperties,
    val mqttService: MqttService<String>
) {

    fun handle(path: Path) {
        val pathJson: JsonObject = Gson().toJsonTree(path).asJsonObject
        log.info(MqttConnectionType.FROM_RMS, "path : [$path]")
        log.info(MqttConnectionType.FROM_RMS, "pathJson : [$pathJson]")
        mqttService.bridge(
            MqttConnectionType.TO_BRIDGE,
            topic = "/test/path",
            pathJson.toString()
        )
    }
}