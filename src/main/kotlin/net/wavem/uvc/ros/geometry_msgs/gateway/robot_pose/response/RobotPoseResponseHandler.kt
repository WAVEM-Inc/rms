package net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.response

import net.wavem.uvc.mqtt.infra.MqttConfiguration
import net.wavem.uvc.mqtt.infra.MqttLogger
import net.wavem.uvc.ros.geometry_msgs.msg.Pose
import org.springframework.stereotype.Component

@Component
class RobotPoseResponseHandler(
    private val log: MqttLogger,
    private val mqttOutboundGateway: MqttConfiguration.MqttOutboundGateway
) {

    fun handle(pose: Pose) {
        log.info(RESPONSE_CLASS_TYPE, "[$MQTT_ROBOT_POSE_RESPONSE_TOPIC] message arrived [$pose]")
        mqttOutboundGateway.publish(topic = MQTT_ROBOT_POSE_RESPONSE_TOPIC, data = pose)
    }

    companion object {
        const val RESPONSE_CLASS_TYPE: String = "RESP"
        const val MQTT_ROBOT_POSE_RESPONSE_TOPIC: String = "/atc/uvc/response/robot_pose"
    }
}