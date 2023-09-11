package net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.response

import net.wavem.uvc.mqtt.application.MqttService
import net.wavem.uvc.mqtt.domain.MqttConnectionType
import net.wavem.uvc.rms.domain.RmsCommonProperties
import net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.domain.RobotPoseProperties
import net.wavem.uvc.ros.geometry_msgs.msg.Pose
import org.springframework.stereotype.Component

@Component
class RobotPoseResponseHandler(
    private val rmsCommonProperties: RmsCommonProperties,
    private val robotPoseProperties: RobotPoseProperties,
    private val mqttService: MqttService<Pose>
) {

    fun handle(pose: Pose) {
        mqttService.handle(
            connectionType = MqttConnectionType.RESP,
            topic = rmsCommonProperties.fromRosTopicFormat + robotPoseProperties.topic,
            data = pose
        )
    }
}