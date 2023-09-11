package net.wavem.uvc.ros.geometry_msgs.msg

data class Pose(
    val position: Point,
    val orientation: Quaternion
) {
}