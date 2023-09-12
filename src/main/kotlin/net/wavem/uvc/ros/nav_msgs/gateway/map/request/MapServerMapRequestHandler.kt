package net.wavem.uvc.ros.nav_msgs.gateway.map.request

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.nav_msgs.gateway.map.domain.MapServerMapProperties
import net.wavem.uvc.ros.nav_msgs.srv.GetMap
import org.springframework.stereotype.Component

@Component
class MapServerMapRequestHandler(
    private val mapServerMapProperties: MapServerMapProperties,
    private val mqttService: MqttService<GetMap>
) {

    fun handle(getMap: GetMap) {
        mqttService.bridge(
            connectionType = MqttConnectionType.TO_BRIDGE,
            topic = mapServerMapProperties.toBridge,
            data = getMap
        )
    }
}