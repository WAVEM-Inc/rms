package net.wavem.uvc.ros.std_msgs.msg

import net.wavem.uvc.ros.builtin_interfaces.msg.Time

data class Header(
    val seq: Int,
    val stamp: Time,
    val frame_id: String
) {
}