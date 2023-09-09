package net.wavem.uvc.ros2.nav_msgs.msg

import net.wavem.uvc.ros2.std_msgs.msg.Header

data class OccupancyGrid(
    val header: Header,
    val info: MapMetaData,
    val data: IntArray
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as OccupancyGrid

        if (!data.contentEquals(other.data)) return false

        return true
    }

    override fun hashCode(): Int {
        return data.contentHashCode()
    }
}