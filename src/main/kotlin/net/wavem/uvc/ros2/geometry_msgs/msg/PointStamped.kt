package net.wavem.uvc.ros2.geometry_msgs.msg

import net.wavem.uvc.ros2.std_msgs.msg.Header

data class PointStamped(
    val header: Header,
    val point: Point
) {
}