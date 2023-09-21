package net.wavem.uvc.rms.gateway.control.request

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.control.domain.Control
import net.wavem.uvc.rms.gateway.control.domain.ControlProperties
import net.wavem.uvc.rms.gateway.control.domain.cmd.ControlCmd
import org.springframework.stereotype.Component

@Component
class ControlRequestHandler(
    val log : MqttLogger,
    val controlProperties : ControlProperties,
    val mqttService : MqttService<String>,
    val gson : Gson
) {

    fun handle(control : Control) {
        val controlJson : JsonObject = gson.toJsonTree(control).asJsonObject
        log.info(MqttConnectionType.FROM_RMS, "controlJson : [$controlJson]")

        val header : Header? = control.header
        val controlCmd : ControlCmd? = control.controlCmd

        if(header == null) {
            log.error(MqttConnectionType.FROM_RMS, "control header is null skipping...")
            return
        } else if (controlCmd == null) {
            log.error(MqttConnectionType.FROM_RMS, "control controlCmd is null skipping...")
            return
        }

        val headerJson : JsonObject = controlJson.getAsJsonObject(KEY_HEADER)
        log.info(MqttConnectionType.FROM_RMS, "control headerJson : [$headerJson]")

        val controlCmdJson : JsonObject = controlJson.getAsJsonObject(KEY_CONTROL_CMD)
        log.info(MqttConnectionType.FROM_RMS, "control controlCmdJson : [$controlCmdJson]")

        mqttService.bridge(
            MqttConnectionType.TO_BRIDGE,
            "/test/control",
            controlJson.toString()
        )
    }

    private companion object {
        const val KEY_HEADER : String = "header"
        const val KEY_CONTROL_CMD : String = "controlCmd"
    }
}