package net.wavem.uvc.ros.nav_msgs.gateway.odometry.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.nav_msgs.gateway.odometry.domain.OdometryProperties
import net.wavem.uvc.ros.nav_msgs.msg.Odometry
import org.springframework.stereotype.Component

@Component
class OdometryResponseHandler(
    private val odometryProperties: OdometryProperties,
    private val mqttService: MqttService<Odometry>
) {

    fun handle(odometry: Odometry) {
        mqttService.bridge(
            connectionType = MqttConnectionType.TO_BRIDGE,
            topic = odometryProperties.fromBridge,
            data = odometry
        )
    }
}