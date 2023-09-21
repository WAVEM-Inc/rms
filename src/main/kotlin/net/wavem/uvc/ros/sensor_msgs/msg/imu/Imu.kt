package net.wavem.uvc.ros.sensor_msgs.msg.imu

import net.wavem.uvc.ros.geometry_msgs.msg.quaternion.Quaternion
import net.wavem.uvc.ros.geometry_msgs.msg.vector3.Vector3
import net.wavem.uvc.ros.std_msgs.msg.header.Header

data class Imu(
    val header: Header,
    val orientation: Quaternion,
    val orientation_covariance: DoubleArray = DoubleArray(9),
    val angular_velocity: Vector3,
    val angular_velocity_covariance: DoubleArray = DoubleArray(9),
    val linear_acceleration: Vector3,
    val linear_acceleration_covariance: DoubleArray = DoubleArray(9)
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Imu

        if (!orientation_covariance.contentEquals(other.orientation_covariance)) return false
        if (!angular_velocity_covariance.contentEquals(other.angular_velocity_covariance)) return false
        if (!linear_acceleration_covariance.contentEquals(other.linear_acceleration_covariance)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = orientation_covariance.contentHashCode()
        result = 31 * result + angular_velocity_covariance.contentHashCode()
        result = 31 * result + linear_acceleration_covariance.contentHashCode()
        return result
    }
}