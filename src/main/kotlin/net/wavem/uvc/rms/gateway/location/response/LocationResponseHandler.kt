package net.wavem.uvc.rms.gateway.location.response

import com.google.gson.Gson
import com.google.gson.JsonObject
import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.rms.gateway.location.domain.Location
import net.wavem.uvc.ros.std_msgs.gateway.chatter.response.ChatterResponseHandler
import org.springframework.stereotype.Component

@Component
class LocationResponseHandler(
    private val log: MqttLogger,
    private val chatterResponseHandler: ChatterResponseHandler,
    private val mqttService: MqttService<String>,
) {

    fun handle() {
        val chatterString: net.wavem.uvc.ros.std_msgs.msg.String? = chatterResponseHandler.stdString

        if (chatterString != null) {
            log.info(MqttConnectionType.TO_RMS, "location chatterString test result : [$chatterString] =====")
            val location: Location = Location(chatterString)
            val locationJSON: JsonObject = Gson().toJsonTree(location).asJsonObject
            log.info(MqttConnectionType.TO_RMS,"location chatterJSON : [${locationJSON} =====")
            mqttService.bridge(MqttConnectionType.FROM_BRIDGE, "/test/location", locationJSON.toString())
        } else {
            log.error(MqttConnectionType.TO_RMS, "location chatterString test result is null")
            return
        }
    }
}