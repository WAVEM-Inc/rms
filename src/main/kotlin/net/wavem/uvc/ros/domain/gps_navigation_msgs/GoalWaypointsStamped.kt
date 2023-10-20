package net.wavem.uvc.ros.domain.gps_navigation_msgs

import id.jrosmessages.Message
import net.wavem.uvc.ros.domain.geometry_msgs.Pose
import net.wavem.uvc.ros.domain.std_msgs.Header
import java.nio.ByteBuffer
import java.nio.ByteOrder

class GoalWaypointsStamped() : Message {
    var header : Header = Header()
    var goal_waypoints_list : MutableList<Pose> = mutableListOf()

    constructor(header : Header, goal_waypoints_list : MutableList<Pose>) : this() {
        this.header = header
        this.goal_waypoints_list = goal_waypoints_list
    }

    fun write() : ByteArray {
        val goal_waypoints_list_size : Int = this.goal_waypoints_list.size
        val buf : ByteBuffer = ByteBuffer.allocate(this.header.size + 1024)
        buf.order(ByteOrder.LITTLE_ENDIAN)

        println("GoalWaypointsStamped header : ${this.header.toString()}")
        val headerBytes : ByteArray = this.header.write()
        buf.put(headerBytes)

        buf.putInt(goal_waypoints_list_size)
        println("GoalWaypointsStamped goal_waypoints_list size : ${goal_waypoints_list_size}")

        for (pose in this.goal_waypoints_list) {
            println("GoalWaypointsStamped pose : ${pose.toString()}")
            val poseBytes : ByteArray = pose.write()
            buf.put(poseBytes)
            println("GoalWaypointsStamped pose added")
        }

        return buf.array()
    }
}