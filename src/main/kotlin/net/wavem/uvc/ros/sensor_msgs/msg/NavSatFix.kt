package net.wavem.uvc.ros.sensor_msgs.msg

import net.wavem.uvc.ros.std_msgs.msg.Header

data class NavSatFix(
    val header: Header,
    val status: NavSatStatus,
    val latitude: Double,
    val longitude: Double,
    val altitude: Double,
    val position_covariance: DoubleArray = DoubleArray(9),
    val COVARIANCE_TYPE_UNKNOWN: Byte = 0,
    val COVARIANCE_TYPE_APPROXIMATED: Byte = 1,
    val COVARIANCE_TYPE_DIAGONAL_KNOWN: Byte = 2,
    val COVARIANCE_TYPE_TYPE_KNOWN: Byte = 3,
    val position_covariance_type: Byte
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as NavSatFix

        return position_covariance.contentEquals(other.position_covariance)
    }

    override fun hashCode(): Int {
        return position_covariance.contentHashCode()
    }
}