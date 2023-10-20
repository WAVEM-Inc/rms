package net.wavem.uvc.ros.domain.gts_navigation_msgs

import id.jrosmessages.Message
import net.wavem.uvc.ros.domain.sensor_msgs.NavSatFix
import java.nio.ByteBuffer
import java.nio.ByteOrder

class GoalWaypoints() : Message {
    var goal_waypoints_list : MutableList<NavSatFix> = mutableListOf()

    constructor(goal_waypoints_list : MutableList<NavSatFix>) : this() {
        this.goal_waypoints_list = goal_waypoints_list
    }

    fun write() : ByteArray {
        val goal_waypoints_list_size : Int = this.goal_waypoints_list.size
        val buf : ByteBuffer = ByteBuffer.allocate(Int.SIZE_BYTES + 16 + (Byte.SIZE_BYTES + UShort.SIZE_BYTES) + (Double.SIZE_BYTES * 3) + ((Double.SIZE_BYTES * 9) * 2) + UByte.SIZE_BYTES)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        buf.putInt(goal_waypoints_list_size)

        for ((index, navSatFix) in this.goal_waypoints_list.withIndex()) {
            val navSatFixBytes : ByteArray = navSatFix.write()
            val navSatFixBytesSize : Int = navSatFixBytes.size

            if (index == 0) {
                buf.putInt(navSatFixBytesSize * goal_waypoints_list_size)
            }
            buf.put(navSatFixBytes)
        }

        return buf.array()
    }
}