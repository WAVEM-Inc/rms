package net.wavem.uvc.ros2.nav_msgs.api.map.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros2.nav_msgs.srv.GetMap
import org.springframework.stereotype.Component

@Component
class MapServerMapResponseHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {

    fun handle(getMap: GetMap) {
        log.info(RESPONSE_CLASS_TYPE, "[${MQTT_MAP_SERVER_MAP_RESPONSE_TOPIC}] message arrived [$getMap]")
        mqttOutboundGateway.publish(topic = MQTT_MAP_SERVER_MAP_RESPONSE_TOPIC, data = getMap)
    }

    companion object {
        const val RESPONSE_CLASS_TYPE: kotlin.String = "RESP"
        const val MQTT_MAP_SERVER_MAP_RESPONSE_TOPIC: String = "/atc/uvc/response/map_server/map"
    }
}