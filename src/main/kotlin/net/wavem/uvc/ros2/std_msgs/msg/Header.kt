package net.wavem.uvc.ros2.std_msgs.msg

import net.wavem.uvc.ros2.builtin_interfaces.msg.Time

data class Header(
    val seq: Int,
    val stamp: Time,
    val frame_id: String
) {
}