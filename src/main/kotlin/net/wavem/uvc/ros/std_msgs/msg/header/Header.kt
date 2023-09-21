package net.wavem.uvc.ros.std_msgs.msg.header

import net.wavem.uvc.ros.builtin_interfaces.msg.Time
import net.wavem.uvc.ros.std_msgs.msg.string.String

data class Header(
    val seq : Int,
    val stamp : Time,
    val frame_id : String
) {
}