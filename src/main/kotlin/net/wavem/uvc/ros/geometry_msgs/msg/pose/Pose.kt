package net.wavem.uvc.ros.geometry_msgs.msg.pose

import net.wavem.uvc.ros.geometry_msgs.msg.quaternion.Quaternion
import net.wavem.uvc.ros.geometry_msgs.msg.point.Point

data class Pose(
    val position : Point,
    val orientation : Quaternion
) {
}