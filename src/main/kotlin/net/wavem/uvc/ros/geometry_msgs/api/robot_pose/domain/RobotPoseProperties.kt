package net.wavem.uvc.ros.geometry_msgs.api.robot_pose.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("robotpose")
data class RobotPoseProperties(
    val responseFromBridgeTopic: String,
    val qos: Int
) {
}