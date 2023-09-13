package net.wavem.uvc.rms.gateway.event.gateway

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.gateway.event.domain.Event
import net.wavem.uvc.rms.gateway.event.domain.EventProperties
import org.springframework.stereotype.Component

@Component
class EventRequestHandler(
    private val log: MqttLogger,
    private val eventProperties: EventProperties,
    private val mqttService: MqttService<String>
) {

    fun handle(event: Event) {
        val eventJson: JsonObject = Gson().toJsonTree(event).asJsonObject
        log.info(MqttConnectionType.FROM_RMS, "eventJson : [$eventJson]")
        mqttService.bridge(
            MqttConnectionType.TO_BRIDGE,
            topic = "/test/event",
            qos = 0,
            eventJson.toString()
        )
    }
}