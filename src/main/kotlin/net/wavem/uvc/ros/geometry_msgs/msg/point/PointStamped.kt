package net.wavem.uvc.ros.geometry_msgs.msg.point

import net.wavem.uvc.ros.std_msgs.msg.header.Header

data class PointStamped(
    val header: Header,
    val point: Point
) {
}