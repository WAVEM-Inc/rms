package net.wavem.uvc.ros.nav_msgs.msg

import net.wavem.uvc.ros.geometry_msgs.msg.PoseWithCovariance
import net.wavem.uvc.ros.geometry_msgs.msg.TwistWithCovariance
import net.wavem.uvc.ros.std_msgs.msg.Header

data class Odometry(
    val header: Header,
    val child_frame_id: String,
    val pose: PoseWithCovariance,
    val twist: TwistWithCovariance
) {
}