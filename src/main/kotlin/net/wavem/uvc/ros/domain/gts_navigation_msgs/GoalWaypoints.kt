package net.wavem.uvc.ros.domain.gts_navigation_msgs

import id.jrosmessages.Message
import net.wavem.uvc.ros.domain.geometry_msgs.Point
import java.nio.ByteBuffer
import java.nio.ByteOrder

class GoalWaypoints() : Message {
    var goal_waypoints_list : MutableList<Point> = mutableListOf()

    constructor(goal_waypoints_list : MutableList<Point>) : this() {
        this.goal_waypoints_list = goal_waypoints_list
    }

    fun write() : ByteArray {
        val goal_waypoints_list_size : Int = this.goal_waypoints_list.size
        val buf : ByteBuffer = ByteBuffer.allocate((Int.SIZE_BYTES * 2) + (Double.SIZE_BITS * 3))
        buf.order(ByteOrder.LITTLE_ENDIAN)

        buf.putInt(goal_waypoints_list_size)

        for ((index, point) in this.goal_waypoints_list.withIndex()) {
            println("GoalWaypoints navSatFix : ${point.toString()}")

            val pointBytes : ByteArray = point.write()
            val pointBytesSize : Int = pointBytes.size

            if (index == 0) {
                buf.putInt(pointBytesSize * goal_waypoints_list_size)
            }
            buf.put(pointBytes)
        }

        return buf.array()
    }
}