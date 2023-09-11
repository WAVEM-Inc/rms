package net.wavem.uvc.ros.geometry_msgs.msg

import net.wavem.uvc.ros.std_msgs.msg.Header

data class PointStamped(
    val header: Header,
    val point: Point
) {
}