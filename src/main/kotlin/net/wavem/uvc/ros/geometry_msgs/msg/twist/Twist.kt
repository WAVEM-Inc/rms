package net.wavem.uvc.ros.geometry_msgs.msg.twist

import net.wavem.uvc.ros.geometry_msgs.msg.vector3.Vector3

data class Twist(
    val linear: Vector3,
    val angular: Vector3
) {
}