package net.wavem.uvc.ros.geometry_msgs.gateway.robot_pose.domain

import org.springframework.boot.context.properties.ConfigurationProperties

@ConfigurationProperties("robotpose")
data class RobotPoseProperties(
    val default: String,
    val fromBridge: String,
    val qos: Int
) {
}