package net.wavem.uvc.ros.nav_msgs.msg.map

import net.wavem.uvc.ros.geometry_msgs.msg.pose.Pose
import java.util.*

data class MapMetaData(
    val map_load_time : Date,
    val resolution : Float,
    val width : Int,
    val height : Int,
    val origin : Pose
) {
}