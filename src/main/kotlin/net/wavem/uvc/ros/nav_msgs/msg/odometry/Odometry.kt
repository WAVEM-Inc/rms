package net.wavem.uvc.ros.nav_msgs.msg.odometry

import net.wavem.uvc.ros.geometry_msgs.msg.pose.PoseWithCovariance
import net.wavem.uvc.ros.geometry_msgs.msg.twist.TwistWithCovariance
import net.wavem.uvc.ros.std_msgs.msg.header.Header

data class Odometry(
    val header: Header,
    val child_frame_id: String,
    val pose: PoseWithCovariance,
    val twist: TwistWithCovariance
) {
}