package net.wavem.uvc.ros2.geometry_msgs.msg

data class TwistWithCovariance(
    val twist: Twist,
    val covariance: DoubleArray = DoubleArray(36)
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as TwistWithCovariance

        return covariance.contentEquals(other.covariance)
    }

    override fun hashCode(): Int {
        return covariance.contentHashCode()
    }
}