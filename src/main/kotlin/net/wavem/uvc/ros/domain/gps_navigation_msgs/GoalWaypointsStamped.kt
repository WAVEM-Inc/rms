package net.wavem.uvc.ros.domain.gps_navigation_msgs

import id.jrosmessages.Message
import net.wavem.uvc.ros.domain.geometry_msgs.Pose
import net.wavem.uvc.ros.domain.std_msgs.Header
import java.nio.ByteBuffer
import java.nio.ByteOrder

class GoalWaypointsStamped() : Message {
    var header : Header = Header()
    var goal_waypoints : Pose = Pose()
    var goal_waypoints_list : Array<Pose> = arrayOf()

    fun write() : ByteArray {
        val buf : ByteBuffer = ByteBuffer.allocate(Integer.BYTES * 2)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        return buf.array()
    }
}