package net.wavem.uvc.ros2.nav_msgs.msg

import net.wavem.uvc.ros2.geometry_msgs.msg.Pose
import java.util.*

data class MapMetaData(
    val map_load_time: Date,
    val resolution: Float,
    val width: Int,
    val height: Int,
    val origin: Pose
) {
}