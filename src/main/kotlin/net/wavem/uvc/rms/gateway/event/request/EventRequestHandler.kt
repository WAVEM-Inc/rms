package net.wavem.uvc.rms.gateway.event.request

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.common.domain.header.Header
import net.wavem.uvc.rms.gateway.event.domain.Event
import net.wavem.uvc.rms.gateway.event.domain.EventProperties
import net.wavem.uvc.rms.gateway.event.domain.com_info.ComInfo
import net.wavem.uvc.rms.gateway.event.domain.event_info.EventInfo
import net.wavem.uvc.rms.gateway.event.domain.job.EventJobInfo
import org.springframework.stereotype.Component

@Component
class EventRequestHandler(
    private val log: MqttLogger,
    private val eventProperties: EventProperties,
    private val mqttService: MqttService<String>,
    val gson: Gson
) {

    fun handle(event: Event) {
        val eventJson: JsonObject = gson.toJsonTree(event).asJsonObject
        log.info(MqttConnectionType.FROM_RMS, "eventJson : [$eventJson]")

        val header: Header? = event.header
        val jobInfo: EventJobInfo? = event.jobInfo
        val eventInfo: EventInfo? = event.eventInfo
        val comInfo: ComInfo? = event.comInfo

        if(header == null) {
            log.error(MqttConnectionType.FROM_RMS, "event header is null skipping...")
            return
        } else if (jobInfo == null) {
            log.error(MqttConnectionType.FROM_RMS, "event jobInfo is null skipping...")
            return
        } else if(eventInfo == null) {
            log.error(MqttConnectionType.FROM_RMS, "event eventInfo is null skipping...")
            return
        } else if (comInfo == null) {
            log.error(MqttConnectionType.FROM_RMS, "event comInfo is null skipping...")
            return
        }

        val headerJson: JsonObject = eventJson.getAsJsonObject(KEY_HEADER)
        log.info(MqttConnectionType.FROM_RMS, "event headerJson : [$headerJson]")

        val jobInfoJson: JsonObject = eventJson.getAsJsonObject(KEY_JOB_INFO)
        log.info(MqttConnectionType.FROM_RMS, "event jobInfoJson : [$jobInfoJson]")

        val eventInfoJson: JsonObject = eventJson.getAsJsonObject(KEY_EVENT_INFO)
        log.info(MqttConnectionType.FROM_RMS, "event eventInfoJson : [$eventInfoJson]")

        val comInfoJson: JsonObject = eventJson.getAsJsonObject(KEY_COM_INFO)
        log.info(MqttConnectionType.FROM_RMS, "event comInfoJson : [$comInfoJson]")

        mqttService.bridge(
            MqttConnectionType.TO_BRIDGE,
            topic = "/test/event",
            eventJson.toString()
        )
    }

    private companion object {
        const val KEY_HEADER: String = "header"
        const val KEY_JOB_INFO: String = "jobInfo"
        const val KEY_EVENT_INFO: String = "eventInfo"
        const val KEY_COM_INFO: String = "comInfo"
    }
}