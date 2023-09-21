package net.wavem.uvc.ros.nav_msgs.msg.map

import net.wavem.uvc.ros.std_msgs.msg.header.Header

data class OccupancyGrid(
    val header : Header,
    val info : MapMetaData,
    val data : ByteArray
) {
    override fun equals(other : Any?) : Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as OccupancyGrid

        return data.contentEquals(other.data)
    }

    override fun hashCode() : Int {
        return data.contentHashCode()
    }

}