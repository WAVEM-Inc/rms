package net.wavem.uvc.ros2.nav_msgs.api.map.request

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.ros2.geometry_msgs.msg.Pose
import org.springframework.stereotype.Component

@Component
class MapRequestHandler(
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {
    fun handle(pose: Pose) {

    }

    companion object {
        const val MQTT_CMD_VEL_REQUEST_TOPIC: String = "/request/cmd_vel"
    }
}