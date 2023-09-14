package net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.domain.RobotPoseProperties
import net.wavem.uvc.ros.geometry_msgs.msg.pose.Pose
import org.springframework.stereotype.Component

@Component
class RobotPoseResponseHandler(
    private val robotPoseProperties: RobotPoseProperties,
    private val mqttService: MqttService<Pose>
) {

    fun handle(pose: Pose) {
        mqttService.bridge(
            connectionType = MqttConnectionType.FROM_BRIDGE,
            topic = robotPoseProperties.fromBridge,
            data = pose
        )
    }
}