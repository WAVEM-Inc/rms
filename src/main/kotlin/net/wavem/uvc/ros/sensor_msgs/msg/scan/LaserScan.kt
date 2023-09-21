package net.wavem.uvc.ros.sensor_msgs.msg.scan

import net.wavem.uvc.ros.std_msgs.msg.header.Header

data class LaserScan(
    val header : Header,
    val angle_min : Float,
    val angle_max : Float,
    val angle_increment : Float,
    val time_increment : Float,
    val scan_time : Float,
    val range_time : Float,
    val range_max : Float,
    val ranges : FloatArray,
    val intensities : FloatArray
) {
    override fun equals(other : Any?) : Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as LaserScan

        if (!ranges.contentEquals(other.ranges)) return false
        if (!intensities.contentEquals(other.intensities)) return false

        return true
    }

    override fun hashCode() : Int {
        var result = ranges.contentHashCode()
        result = 31 * result + intensities.contentHashCode()
        return result
    }
}