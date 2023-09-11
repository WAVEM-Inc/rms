package net.wavem.uvc.ros.nav_msgs.gateway.map.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.rms.domain.RmsCommonProperties
import net.wavem.uvc.ros.nav_msgs.gateway.map.domain.MapServerMapProperties
import net.wavem.uvc.ros.nav_msgs.srv.GetMap
import org.springframework.stereotype.Component

@Component
class MapServerMapResponseHandler(
    private val rmsCommonProperties: RmsCommonProperties,
    private val mapServerMapProperties: MapServerMapProperties,
    private val mqttService: MqttService<GetMap>
) {

    fun handle(getMap: GetMap) {
        mqttService.bridge(
            connectionType = MqttConnectionType.REQ,
            topic = mapServerMapProperties.fromBridge,
            data = getMap
        )
    }
}