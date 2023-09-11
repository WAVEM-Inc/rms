package net.wavem.uvc.ros.nav_msgs.gateway.map.request

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttHandler
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.nav_msgs.srv.GetMap
import org.springframework.stereotype.Component

@Component
class MapServerMapRequestHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) : MqttHandler<GetMap> {
    override fun handle(data: GetMap) {
        log.info(REQUEST_CLASS_TYPE, "[${MQTT_MAP_SERVER_MAP_REQUEST_TOPIC}] message arrived [$data]")
        mqttOutboundGateway.publish(topic = MQTT_MAP_SERVER_MAP_REQUEST_TOPIC, data = data)
    }

    companion object {
        const val REQUEST_CLASS_TYPE: String = "REQ"
        const val MQTT_MAP_SERVER_MAP_REQUEST_TOPIC: String = "/request/map_server/map"
    }
}